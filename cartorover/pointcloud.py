import io

import numpy as np


class PointCloud:
    """A quick and dirty reader for the PCD format:
    https://pointclouds.org/documentation/tutorials/pcd_file_format.html

    Hard-coded for what comes out of the VIAM interface for the RP LIDAR A3
    https://www.slamtec.ai/home/rplidar_a3/
    """

    def __init__(
        self,
        version: bytes = b"",
        fields: bytes = b"",
        size: bytes = b"",
        type: bytes = b"",
        count: bytes = b"",
        width: bytes = b"",
        height: bytes = b"",
        viewpoint: bytes = b"",
        points: bytes = b"",
        data: bytes = b"",
    ):
        self.version = version
        self.fields = fields
        self.size = size
        self.type = type
        self.count = count
        self.width = width
        self.height = height
        self.viewpoint = viewpoint
        self.points = points
        self.data = data

    @classmethod
    def from_bytes(cls, data: bytes) -> "PointCloud":
        bf = io.BytesIO(data)
        pc = PointCloud()

        # Assign the headers. "DATA" is the last header.
        k, v = b"", b""
        while k != b"DATA":
            k, v = bf.readline().strip().split(b" ", 1)
            setattr(pc, k.decode().lower(), v)

        # Read the data into a np array.  We assume we're getting x,y,z points
        # as floats.
        pc.cloud = np.frombuffer(bf.read(), np.dtype([('x', 'f'), ('y', 'f'), ('z', 'f')]))
        return pc

    def closest_point(self) -> tuple[float, np.ndarray]:
        dist = np.sqrt(np.power(self.cloud['x'], 2) + np.power(self.cloud['y'], 2))
        return dist.min(), self.cloud[np.where(dist == dist.min())[0]]
