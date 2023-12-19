class Drivetrain:
    front_left_drive_motor: wpilib.CANSparkMax
    front_right_drive_motor: wpilib.CANSparkMax
    back_left_drive_motor: wpilib.CANSparkMax
    back_right_drive_motor: wpilib.CANSparkMax

    left_motor_controller_group: wpilib.MotorControllerGroup
    right_motor_controller_group: wpilib.MotorControllerGroup

    drive_train: wpilib.DifferentialDrive

    def execute():
        pass