from collections import namedtuple

# Configuration objects will be injected into component classes
DrivetrainConfig = namedtuple(
    "DrivetrainConfig",
    "front_left_id front_right_id back_left_id back_right_id controller_type",
)
pandemonium_cfg = DrivetrainConfig(
    front_left_id=11,
    front_right_id=12,
    back_left_id=21,
    back_right_id=22,
    controller_type="SPARK_MAX",
)
pancake_cfg = DrivetrainConfig(
    front_left_id=8,
    front_right_id=7,
    back_left_id=9,
    back_right_id=11,
    controller_type="TALON",
)
