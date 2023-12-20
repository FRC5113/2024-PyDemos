import wpilib
from rev import CANSparkMax
from magicbot import will_reset_to

class Drivetrain:

    # annotate motor and motor controller objects
    front_left_drive_motor: CANSparkMax
    front_right_drive_motor: CANSparkMax
    back_left_drive_motor: CANSparkMax
    back_right_drive_motor: CANSparkMax

    left_motor_controller_group: wpilib.MotorControllerGroup
    right_motor_controller_group: wpilib.MotorControllerGroup

    drive_train: wpilib.DifferentialDrive

    # values will reset to 0 after every time control loop runs
    forward = will_reset_to(0)
    turn = will_reset_to(0)

    def setup(self):
        self.left_motor_controller_group = wpilib.MotorControllerGroup(self.front_left_drive_motor,self.back_left_drive_motor)
        self.right_motor_controller_group = wpilib.MotorControllerGroup(self.front_right_drive_motor,self.back_right_drive_motor)
        self.drivetrain = DifferentialDrive(self.left_motor_controller_group,self.right_motor_controller_group)


    def arcade_drive(self, f, t)
        self.forward = f
        self.turn = t

    def execute(self):
        self.drive_train.arcadeDrive(self.forward, self.turn)