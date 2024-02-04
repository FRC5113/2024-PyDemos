import wpilib
from wpilib.interfaces import MotorController
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to
from rev import CANSparkBase
from phoenix6.signals import NeutralModeValue

from config import ControllerType


class Drivetrain:
    # annotate motor and configuration instances
    front_left_motor: MotorController
    front_right_motor: MotorController
    back_left_motor: MotorController
    back_right_motor: MotorController

    controller_type: ControllerType

    # values will reset to 0 after every time control loop runs
    forward = will_reset_to(0)
    turn = will_reset_to(0)

    def setup(self):
        self.left_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_left_motor, self.back_left_motor
        )
        self.right_motor_controller_group = wpilib.MotorControllerGroup(
            self.front_right_motor, self.back_right_motor
        )
        self.right_motor_controller_group.setInverted(True)
        self.drive = DifferentialDrive(
            self.left_motor_controller_group, self.right_motor_controller_group
        )
        idle_mode = None
        if self.controller_type == ControllerType.SPARK_MAX:
            idle_mode = CANSparkBase.IdleMode.kCoast
        elif self.controller_type == ControllerType.TALON_FX:
            idle_mode = NeutralModeValue.COAST
        # duck typing go brrrr
        self.front_left_motor.setIdleMode(idle_mode)
        self.front_right_motor.setIdleMode(idle_mode)
        self.back_left_motor.setIdleMode(idle_mode)
        self.back_right_motor.setIdleMode(idle_mode)


    def arcade_drive(self, forward: float, turn: float):
        assert -1.0 < forward < 1.0, f"Improper forward: {forward}"
        assert -1.0 < turn < 1.0, f"Improper turn: {turn}"
        self.forward = forward
        self.turn = turn

    def execute(self):
        self.drive.arcadeDrive(self.forward, self.turn)
