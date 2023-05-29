"""Module makes a rover follow a line using a webcam for color detection."""

import asyncio
import os

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions
from viam.components.base import Base, Vector3
from viam.components.camera import Camera
from viam.services.vision import VisionClient

# States
DRIVE = "drive"
APPROACHING = "approaching"
CLOSED_TO_BAGEL = "closed_to_bagel"
STATION = "station"

# Fine tuning
ANGULAR_POWER = 0.25
LINEAR_POWER = 0.35

# Bagel detection parameters
BAGEL_DETECTION_CLASSES = ["59", "60"]
BAGEL_DETECTION_CONFIDENCE = 0.55
# CROP = {
#     "FRONT": (x * 0.4, y * 0.75, x * 0.6, y),
# }


class LostError(Exception):
    pass


async def main():
    robot = await connect(os.environ["ROBOT_SECRET"], os.environ["ROBOT_LOCATION"])

    try:
        camera = Camera.from_robot(robot, "cam")
        base = Base.from_robot(robot, "viam_base")

        green_vision = VisionClient.from_robot(robot, "green_detector")
        pink_vision = VisionClient.from_robot(robot, "pink_detector")
        bagel_vision = VisionClient.from_robot(robot, "bagel_detector")
        lost_count = 0
        lost_threshold = 3
        state = DRIVE

        while True:
            frame = await camera.get_image(mime_type="image/jpeg")

            if state == DRIVE:
                found = await drive(base, frame, green_vision)
                if await is_approaching_station(frame, pink_vision):
                    state = APPROACHING
                if await is_detecting_bagel(frame, bagel_vision):
                    state = CLOSED_TO_BAGEL
            elif state == APPROACHING:
                found = await drive(base, frame, green_vision)
                if not await is_approaching_station(frame, pink_vision):
                    state = STATION
            elif state == STATION:
                await stop_robot(robot)
                await asyncio.sleep(4)
                state = DRIVE
            elif state == CLOSED_TO_BAGEL:
                print("Bagel detected!")
                # TODO make it get closer and pick the bagel
                await stop_robot(robot)
                await asyncio.sleep(4)
                state = DRIVE

            if found:
                lost_count = 0
            else:
                lost_count += 1
                if lost_count > lost_threshold:
                    raise LostError()

    except KeyboardInterrupt:
        pass

    finally:
        await stop_robot(robot)
        await robot.close()


async def drive(base, frame, green_vision):
    if await is_color_in_front(frame, green_vision):
        await base.set_power(Vector3(y=LINEAR_POWER), Vector3())
        return True

    if await is_color_there(frame, green_vision, "left"):
        await base.set_power(Vector3(), Vector3(z=ANGULAR_POWER))
        return True

    if await is_color_there(frame, green_vision, "right"):
        await base.set_power(Vector3(), Vector3(z=-ANGULAR_POWER))
        return True

    return False


async def connect(secret, domain):
    # viam1
    creds = Credentials(type="robot-location-secret", payload=secret)
    opts = RobotClient.Options(
        refresh_interval=0, dial_options=DialOptions(credentials=creds)
    )
    return await RobotClient.at_address(domain, opts)


async def is_color_in_front(frame, vis):
    """
    Returns whether the appropriate path color is detected in front of the center of the robot.
    """
    x, y = frame.size[0], frame.size[1]

    # Crop the image to get only the middle fifth of the top third of the original image
    cropped_frame = frame.crop((x * 0.4, y * 0.75, x * 0.6, y))

    detections = await vis.get_detections(cropped_frame)  # , detector_name)
    # print(f"front: {detections}")

    return detections != []


async def is_color_there(frame, vis, location):
    """
    Returns whether the appropriate path color is detected to the left/right of the robot's front.
    """
    # frame = await camera.get_image(mime_type="image/jpeg")
    x, y = frame.size[0], frame.size[1]

    if location == "left":
        # Crop image to get only the left two fifths of the original image
        cropped_frame = frame.crop((0, y * 0.5, x * 0.45, y))

    elif location == "right":
        # Crop image to get only the right two fifths of the original image
        cropped_frame = frame.crop((x * 0.55, y * 0.5, x, y))

    detections = await vis.get_detections(cropped_frame)  # , detector_name)

    # print(f"{location}: {detections}")
    return detections != []


async def is_approaching_station(frame, vis):
    x, y = frame.size[0], frame.size[1]
    cropped_frame = frame.crop((0, y * 0.9, x, y))
    detections = await vis.get_detections(cropped_frame)
    return detections != []


async def is_detecting_bagel(frame, vis):
    detections = await vis.get_detections(frame)
    return [detection for detection in detections if detection.class_name in BAGEL_DETECTION_CLASSES and detection.confidence > BAGEL_DETECTION_CONFIDENCE] != []


async def stop_robot(robot):
    """
    Stop the robot's motion.
    """
    base = Base.from_robot(robot, "viam_base")
    await base.stop()


if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
