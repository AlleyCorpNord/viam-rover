import asyncio
import logging
import os
from contextlib import asynccontextmanager

from viam.logging import LOGGERS
from viam.robot.client import RobotClient
from viam.rpc.dial import Credentials, DialOptions

# Clear out viam's intrusive logging config.
for logname in list(LOGGERS.keys()):
    logging.getLogger(logname).handlers.clear()
    del LOGGERS[logname]

log = logging.getLogger(__name__)


@asynccontextmanager
async def connect(secret: str = "", location: str = "", auth_entity: str = ""):
    """Creates a context to handle connecting/disconnecting the robot
    according to the config.

    Any config values not supplied are sourced from the env.  If any required
    values are not found we raise an error.

    :param secret: defaults to environ["ROBOT_SECRET"].
    :param location: defaults to environ["ROBOT_LOCATION"].
    :param auth_entity: use this value to skip the RTC stuff and connect to
        the local instance of viam-server. defaults to
        environ["ROBOT_AUTH_ENTITY"].
    :raise: RuntimeError
    """
    try:
        secret = secret or os.environ["ROBOT_SECRET"]
        location = location or os.environ["ROBOT_LOCATION"]
        auth_entity = auth_entity or os.environ.get("ROBOT_AUTH_ENTITY")
    except KeyError:
        raise RuntimeError("Failed to configure robot connection.")

    dial_options = DialOptions(
        credentials=Credentials(type="robot-location-secret", payload=secret)
    )

    # Configure local access if ROBOT_AUTH_ENTITY is provided in the env.
    if auth_entity:
        dial_options.auth_entity = auth_entity
        dial_options.disable_webrtc = True

    opts = RobotClient.Options(refresh_interval=0, dial_options=dial_options)
    log.info(f"Connecting to robot {location}")
    try:
        robot = await RobotClient.at_address(location, opts)
        try:
            yield robot
        finally:
            log.info(f"Closing {location}")
            await robot.close()
    except (KeyboardInterrupt, asyncio.exceptions.CancelledError):
        yield
