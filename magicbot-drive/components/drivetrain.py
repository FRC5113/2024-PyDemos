import wpilib
from wpilib.drive import DifferentialDrive
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import will_reset_to

from config import DrivetrainConfig


class Drivetrain:
    # annotate motor and configuration instances
    cfg: DrivetrainConfig

    # values will reset to 0 after every time control loop runs
    forward = will_reset_to(0)
    turn = will_reset_to(0)

    def setup(self):
        if self.cfg.controller_type == "SPARK_MAX":
            self.front_left_drive_motor = CANSparkMax(
                self.cfg.front_left_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.front_right_drive_motor = CANSparkMax(
                self.cfg.front_right_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.back_left_drive_motor = CANSparkMax(
                self.cfg.back_left_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.back_right_drive_motor = CANSparkMax(
                self.cfg.back_right_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
        else:
            raise Exception("Improper controller type passed in drivetrain_cfg")

        self.left_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_left_drive_motor, self.back_left_drive_motor
        )
        self.right_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_right_drive_motor, self.back_right_drive_motor
        )
        self.right_motor_controller_group.setInverted(True)
        self.drive = DifferentialDrive(
            self.left_motor_controller_group, self.right_motor_controller_group
        )

    def arcade_drive(self, forward: float, turn: float):
        if not (-1.0 <= forward <= 1.0):
            raise Exception(f"Improper value for forward entered: {forward}")
            return
        if not (-1.0 <= turn <= 1.0):
            raise Exception(f"Improper value for turn entered: {turn}")
            return
        self.forward = forward
        self.turn = turn

    def execute(self):
        self.drive.arcadeDrive(self.forward, self.turn)
