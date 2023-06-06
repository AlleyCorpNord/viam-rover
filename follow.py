"""Module makes a rover follow a line using a webcam for color detection."""

import asyncio
import os

from itertools import starmap
from operations import mul

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

# Bagel detection parameters
BAGEL_DETECTION_CLASSES = ("59", "60")
BAGEL_DETECTION_CONFIDENCE = 0.55

# Crops
CROP_LOOK_AHEAD = (0.4, 0.75, 0.6, 1)
CROP_LOOK_LEFT = (0, 0.5, 0.45, 1)
CROP_LOOK_RIGHT = (0.55, 0.5, 1, 1)
CROP_APPROACHING_STATION = (0, 0.9, 1, 1)

# States
DRIVE = "drive"
APPROACHING = "approaching"
CLOSE_TO_BAGEL = "close_to_bagel"
STATION = "station"


class LostError(Exception):
    pass


async def main():
    robot = await connect(os.environ["ROBOT_SECRET"], os.environ["ROBOT_LOCATION"])

    try:
        camera = Camera.from_robot(robot, "cam")
        base = Base.from_robot(robot, "viam_base")

        track_vision = VisionClient.from_robot(robot, "green_detector")
        station_vision = VisionClient.from_robot(robot, "pink_detector")
        bagel_vision = VisionClient.from_robot(robot, "bagel_detector")
        lost_count = 0
        prev_state = None
        state = DRIVE

        while True:
            if state != prev_state:
                print(f"changed to: {state}")
                prev_state = state

            frame = await camera.get_image(mime_type="image/jpeg")

            if state == DRIVE:
                found = await drive(base, frame, track_vision)
                if await is_approaching_station(frame, station_vision):
                    state = APPROACHING
                if await is_detecting_bagel(frame, bagel_vision):
                    state = CLOSE_TO_BAGEL
            elif state == APPROACHING:
                found = await drive(base, frame, track_vision)
                if not await is_approaching_station(frame, station_vision):
                    state = STATION
            elif state == STATION:
                await stop_robot(robot)
                await asyncio.sleep(4)
                state = DRIVE
            elif state == CLOSE_TO_BAGEL:
                print("Bagel detected!")
                # TODO make it get closer and pick the bagel
                await stop_robot(robot)
                await asyncio.sleep(4)
                state = DRIVE
            elif state == LOST:
                found = await drive(base, frame, track_vision, True)

            if found:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count > LOST_THRESHOLD:
                    state = LOST
                elif lost_count > REALLY_LOST_THRESHOLD:
                    raise LostError()

    except KeyboardInterrupt:
        pass

    finally:
        await stop_robot(robot)
        await robot.close()


async def drive(base, frame, track_vision, lost=False):
    if await is_track_in_view(frame, track_vision, CROP_LOOK_AHEAD):
        await base.set_power(Vector3(y=LINEAR_POWER), Vector3())
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


async def connect(secret, domain):
    creds = Credentials(type="robot-location-secret", payload=secret)
    opts = RobotClient.Options(
        refresh_interval=0, dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address(domain, opts)


async def is_track_in_view(frame, vis, crop):
    """Returns whether the track is detected in the portion of ``frame`` as
    determined by ``crop``.
    """
    return any(await detections(vis, frame, crop))


async def is_approaching_station(frame, vis):
    return any(await detections(frame, vis, CROP_APPROACHING_STATION))


async def is_detecting_bagel(frame, vis):
    return any(
        [
            detection.class_name in BAGEL_DETECTION_CLASSES
            and detection.confidence > BAGEL_DETECTION_CONFIDENCE
            for detection in await detections(frame, vis)
        ]
    )


async def detections(vis, frame, crop=(0, 0, 1, 1)):
    vis.get_detections(tuple(starmap(mul, zip(crop, frame.size + frame.size))))


async def stop_robot(robot):
    """Stop the robot's motion."""
    base = Base.from_robot(robot, "viam_base")
    await base.stop()


if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
