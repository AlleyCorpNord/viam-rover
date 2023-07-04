"""Module makes a rover follow a line using a webcam for color detection and avoid obstacles such as power bars."""

import asyncio
import os

from itertools import starmap
from operator import mul

import sounddevice as sd
import soundfile as sf

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.base import Base, Vector3
from viam.components.camera import Camera
from viam.services.vision import VisionClient

# Fine tuning
ANGULAR_POWER = 0.25
LINEAR_POWER = 0.35
LOST_THRESHOLD = 3
REALLY_LOST_THRESHOLD = 20

# Obstacle detection parameters
IMPASSABLE_OBSTACLE_DETECTION_CLASS = "impassable"
IMPASSABLE_OBSTACLE_DETECTION_CONFIDENCE = 0.3
IMPASSABLE_POWER = 0.1
PASSABLE_OBSTACLE_DETECTION_CLASS = "can_pass"
PASSABLE_OBSTACLE_DETECTION_CONFIDENCE = 0.3
PASSABLE_POWER = 0.6

# Crops
CROP_LOOK_AHEAD = (0.4, 0.75, 0.6, 1)
CROP_LOOK_LEFT = (0, 0.5, 0.45, 1)
CROP_LOOK_RIGHT = (0.55, 0.5, 1, 1)
CROP_DETECT_OBSTACLE = (0, 0, 1, 1)
# CROP_APPROACHING_STATION = (0, 0.9, 1, 1)

# States
DRIVE = "drive"
# APPROACHING = "approaching"
LOST= "lost"
IMPASSABLE_OBSTACLE_AHEAD = "impassable_obstacle_ahead"
PASSABLE_OBSTACLE_AHEAD = "passable_obstacle_ahead"
# STATION = "station"


# class LostError(Exception):
    # pass

ROBOT_SECRET = "7lfiues1ijftmj8pmis64k90r84pn3is45al96b1f1t3e0vk"
ROBOT_LOCATION = "viam1-main.yo8hf6zu1g.viam.cloud"

async def main():
    # robot = await connect(os.environ["ROBOT_SECRET"], os.environ["ROBOT_LOCATION"])
    robot = await connect(ROBOT_SECRET, ROBOT_LOCATION)

    try:
        camera = Camera.from_robot(robot, "cam")
        base = Base.from_robot(robot, "viam_base")

        track_vision = VisionClient.from_robot(robot, "green_detector")
        # station_vision = VisionClient.from_robot(robot, "pink_detector")
        obstacle_vision = VisionClient.from_robot(robot, "bagel_detector")
        lost_count = 0
        prev_state = None
        state = DRIVE

        while True:
            if state != prev_state:
                print(f"State changed from {prev_state} to {state}.")
                prev_state = state

            frame = await camera.get_image(mime_type="image/jpeg")

            if state == DRIVE:
                found = await drive(base, frame, track_vision)
                if await is_detecting_obstacle(frame, obstacle_vision, IMPASSABLE_OBSTACLE_DETECTION_CLASS, IMPASSABLE_OBSTACLE_DETECTION_CONFIDENCE):
                    state = IMPASSABLE_OBSTACLE_AHEAD
                elif await is_detecting_obstacle(frame, obstacle_vision, PASSABLE_OBSTACLE_DETECTION_CLASS, PASSABLE_OBSTACLE_DETECTION_CONFIDENCE):
                    state = PASSABLE_OBSTACLE_AHEAD
                # if await is_approaching_station(frame, station_vision):
                #     state = APPROACHING
            # elif state == APPROACHING:
            #     found = await drive(base, frame, track_vision)
            #     if not await is_approaching_station(frame, station_vision):
            #         state = STATION
            # elif state == STATION:
            #     await stop_robot(base)
            #     await asyncio.sleep(4)
            #     play_sound()
            #     state = DRIVE
            elif state == IMPASSABLE_OBSTACLE_AHEAD:
                # found = await drive(base, frame, track_vision, linear_power=IMPASSABLE_POWER)
                # if not await is_detecting_obstacle(frame, obstacle_vision, IMPASSABLE_OBSTACLE_DETECTION_CLASS, IMPASSABLE_OBSTACLE_DETECTION_CONFIDENCE):
                #     state = DRIVE
                await evasive_maneuver(base)
                state = DRIVE
            elif state == PASSABLE_OBSTACLE_AHEAD:
                found = await drive(base, frame, track_vision, linear_power=PASSABLE_POWER)
                if not await is_detecting_obstacle(frame, obstacle_vision, PASSABLE_OBSTACLE_DETECTION_CLASS, PASSABLE_OBSTACLE_DETECTION_CONFIDENCE):
                    state = DRIVE
            elif state == LOST:
                found = await drive(base, frame, track_vision, lost=True)

            if found:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count > REALLY_LOST_THRESHOLD:
                    # raise LostError()
                    break
                elif lost_count > LOST_THRESHOLD:
                    state = LOST

    except KeyboardInterrupt:
        pass
    
    finally:
        await stop_robot(base)
        await robot.close()


async def drive(base, frame, track_vision, lost=False, linear_power=LINEAR_POWER):
    if await is_track_in_view(frame, track_vision, CROP_LOOK_AHEAD):
        await base.set_power(Vector3(y=linear_power), Vector3())
        return True

    if await is_track_in_view(frame, track_vision, CROP_LOOK_LEFT):
        await base.set_power(Vector3(), Vector3(z=ANGULAR_POWER))
        return True

    if await is_track_in_view(frame, track_vision, CROP_LOOK_RIGHT):
        await base.set_power(Vector3(), Vector3(z=-ANGULAR_POWER))
        return True

    # If we didn't find the track and we're lost, keep turning.
    if lost:
        await base.set_power(Vector3(), Vector3(z=-ANGULAR_POWER))
    return False

async def evasive_maneuver(base):
    # Note: this function is trash lol
    await base.spin(angle=90, velocity=90)
    await base.set_power(linear=Vector3(y=LINEAR_POWER), angular=Vector3(z=-ANGULAR_POWER), timeout=5)
    await base.spin(angle=-90, velocity=-90)
    

async def connect(secret, domain):
    creds = Credentials(type="robot-location-secret", payload=secret)
    if "ROBOT_AUTH_ENTITY" in os.environ:
        dial_options=DialOptions(
            disable_webrtc=True,
            auth_entity=os.environ["ROBOT_AUTH_ENTITY"],
            credentials=creds
        )
    else:
        dial_options=DialOptions(credentials=creds)

    opts = RobotClient.Options(
        refresh_interval=0, dial_options=dial_options)

    return await RobotClient.at_address(domain, opts)


async def is_track_in_view(frame, vis, crop):
    """Returns whether the track is detected in the portion of ``frame`` as
    determined by ``crop``.
    """
    return any(await detections(vis, frame, crop) or [])


# async def is_approaching_station(frame, vis):
    # return any(await detections(vis, frame, CROP_APPROACHING_STATION) or [])


async def is_detecting_obstacle(frame, vis, obstacle_detection_class, obstacle_detection_confidence):
    return any(
        [
            (detection.class_name == obstacle_detection_class)
            and (detection.confidence > obstacle_detection_confidence)
            for detection in await detections(vis, frame, CROP_DETECT_OBSTACLE) or []
        ]
    )


async def detections(vis, frame, crop=(0, 0, 1, 1), verbose=False):
    cropped_frame = frame.crop(tuple(starmap(mul, zip(crop, frame.size + frame.size))))
    if verbose:
        print(cropped_frame.size, cropped_frame)
    return await vis.get_detections(cropped_frame)


async def stop_robot(base):
    """Stop the robot's motion."""
    await base.stop()

# def play_sound():
#     try:
#       filename = os.environ["METRO_SOUND_PATH"]
#     except KeyError:
#         print("METRO_SOUND_PATH not set, not playing sound")
#         return
# 
#     # Extract data and sampling rate from file
#     data, fs = sf.read(filename, dtype='float32')
#     sd.play(data, fs)
#     status = sd.wait()

if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
