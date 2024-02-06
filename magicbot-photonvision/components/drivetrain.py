import wpilib
from wpilib.interfaces import MotorController
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to, tunable
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
        """Called after `createObjects()` has been called in the main robot class
        and after all components have been created. Injected variables are not
        initialized by the time this component is initialized, but they are
        by this time this function is called. Therefore, perform further
        initialization here rather than, for example, `__init__()`.
        """
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
        self.drive.setExpiration(0.1)

    def on_enable(self):
        """Called when robot enters autonomous or teleoperated mode"""
        self.drive.setSafetyEnabled(True)

    def arcade_drive(self, forward: float, turn: float):
        if not (-1.0 <= forward <= 1.0):
            raise Exception(f"Improper value for forward entered: {forward}")
            return
        if not (-1.0 <= turn <= 1.0):
            raise Exception(f"Improper value for turn entered: {turn}")
            return
        self.forward = forward
        self.turn = turn

    def stop(self):
        self.forward = 0
        self.turn = 0

    def execute(self):
        self.drive.arcadeDrive(self.forward, self.turn)
