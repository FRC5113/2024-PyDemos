from collections import namedtuple
from enum import Enum


class ControllerType(Enum):
    SPARK_MAX = 0
    TALON_FX = 1
    TALON_SRX = 2


# Configuration objects will be injected into component classes
DrivetrainConfig = namedtuple(
    "DrivetrainConfig",
    "front_left_id front_right_id back_left_id back_right_id controller_type",
)
pandemonium_cfg = DrivetrainConfig(
    front_left_id=15,
    front_right_id=18,
    back_left_id=55,
    back_right_id=12,
    controller_type=ControllerType.TALON_SRX,
)
pancake_cfg = DrivetrainConfig(
    front_left_id=8,
    front_right_id=9,
    back_left_id=7,
    back_right_id=11,
    controller_type=ControllerType.TALON_FX,
)
