import asyncio
import os
from contextlib import asynccontextmanager

from viam.components.camera import Camera
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

from .pointcloud import PointCloud


async def main():

    async with connect(
        os.environ["ROBOT_SECRET"],
        os.environ["ROBOT_LOCATION"],
        os.environ.get("ROBOT_AUTH_ENTITY"),
    ) as robot:
        while True:
            # print(robot.resource_names)
            slam_lidar = Camera.from_robot(robot, "rplidar-component")
            # import ipdb; ipdb.set_trace()
            cloud = await slam_lidar.get_point_cloud()
            pc = PointCloud.frombuffer(cloud)
            print(f"closest point is: {pc.closest_point()}")


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

print("Starting up...")
asyncio.run(main())
print("Done.")
