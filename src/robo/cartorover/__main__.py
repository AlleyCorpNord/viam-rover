import asyncio
import logging

from viam.components.camera import Camera

from ..roboacn.utils import connect
from .pointcloud import PointCloud

log = logging.getLogger("robo.cartorover")


async def main():
    async with connect() as robot:
        slam_lidar = Camera.from_robot(robot, "rplidar-component")

        while True:
            clouddata, mimetype = await slam_lidar.get_point_cloud()
            dist, point = PointCloud.from_bytes(clouddata).closest_point()
            log.info(f"closest point is: {dist} ({point.tolist()})")


asyncio.run(main())
