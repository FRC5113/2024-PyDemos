import wpilib
from rev import CANSparkMax
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.follower import Follower
from phoenix6.signals import InvertedValue
from wpilib.drive import DifferentialDrive
from magicbot import will_reset_to


class SparkMaxDrivetrain:
    # annotate motor and configuration instances
    front_left_motor: CANSparkMax
    front_right_motor: CANSparkMax
    back_left_motor: CANSparkMax
    back_right_motor: CANSparkMax

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

class TalonFXDrivetrain:
    # annotate motor and configuration instances
    front_left_motor: TalonFX
    front_right_motor: TalonFX
    back_left_motor: TalonFX
    back_right_motor: TalonFX

    # values will reset to 0 after every time control loop runs
    forward = will_reset_to(0)
    turn = will_reset_to(0)

    left_out = DutyCycleOut(0, enable_foc=False)
    right_out = DutyCycleOut(0, enable_foc=False)

    def setup(self):
        right_config = TalonFXConfiguration()
        right_config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        self.front_right_motor.configurator.apply(right_config)
        self.back_right_motor.configurator.apply(right_config)

        self.back_left_motor.set_control(Follower(self.front_left_motor.device_id, False))
        self.back_right_motor.set_control(Follower(self.front_right_motor.device_id, False))

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
        speeds = DifferentialDrive.arcadeDriveIK(self.forward, self.turn)
        self.left_out.output = speeds.left
        self.right_out.output = speeds.right
        self.front_left_motor.set_control(self.left_out)
        self.front_right_motor.set_control(self.right_out)
