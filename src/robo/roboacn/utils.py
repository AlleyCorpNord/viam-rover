import asyncio
from contextlib import asynccontextmanager

from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

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
    except (KeyboardInterrupt, asyncio.exceptions.CancelledError):
        pass
    finally:
        await robot.close()
