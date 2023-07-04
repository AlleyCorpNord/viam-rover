import asyncio
import os

from viam.components.camera import Camera

from ..roboacn.utils import connect
from .pointcloud import PointCloud


async def main():
    async with connect(
        os.environ["ROBOT_SECRET"],
        os.environ["ROBOT_LOCATION"],
        os.environ.get("ROBOT_AUTH_ENTITY"),
    ) as robot:
        slam_lidar = Camera.from_robot(robot, "rplidar-component")

        while True:
            clouddata, mimetype = await slam_lidar.get_point_cloud()
            dist, point = PointCloud.from_bytes(clouddata).closest_point()
            print(f"closest point is: {dist} ({point.tolist()})")

print("Starting up...")
asyncio.run(main())
print("Done.")
