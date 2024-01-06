import wpilib
from wpilib.drive import DifferentialDrive
from rev import CANSparkMax
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
