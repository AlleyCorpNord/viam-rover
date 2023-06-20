import asyncio
import os
from contextlib import asynccontextmanager

from viam.components.arm import Arm
from viam.components.camera import Camera
from viam.proto.component.arm import JointPositions
from viam.proto.robot import DiscoveryQuery
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions


async def main():
    async with connect(
        os.environ["ROBOT_SECRET"],
        os.environ["ROBOT_LOCATION"],
        os.environ.get("ROBOT_AUTH_ENTITY"),
    ) as robot:
        # print(robot.resource_names)

        # my_arm_component = Arm.from_robot(robot, "beefy")
        # my_arm_end_position = await my_arm_component.get_end_position()
        # print(f"myArm get_end_position return value: {my_arm_end_position}")

        cam = Camera.from_robot(robot, "armcam")
        cam_return_value = await cam.get_image()
        print(f"cam get_image return value: {cam_return_value}")


@asynccontextmanager
async def connect(secret: str, domain: str, auth_entity: str | None):
    creds = Credentials(type="robot-location-secret", payload=secret)

    dial_options = DialOptions(credentials=creds)

    # Configure local access if ROBOT_AUTH_ENTITY is provided in the env.
    if auth_entity:
        dial_options.auth_entity = auth_entity
        dial_options.disable_webrtc = True

    opts = RobotClient.Options(refresh_interval=0, dial_options=dial_options)
    robot = await RobotClient.at_address(domain, opts)

    try:
        yield robot
    finally:
        await robot.close()


if __name__ == "__main__":
    print("Starting up...")
    asyncio.run(main())
    print("Done.")
