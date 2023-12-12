#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
    specifically it contains the code necessary to operate a robot with
    tank drive.
"""

import wpilib
from wpilib.drive import DifferentialDrive
from ctre import WPI_TalonFX
from rev import CANSparkMax

# CAN ids of three of our robots
pancake_ids = {
    "front_left" : 8,
    "back_left" : 7,
    "front_right" : 9,
    "back_right" : 11,
    "controller_type" : "talon"
}
perry_ids = {
    "front_left" : 41,
    "back_left" : 42,
    "front_right" : 44,
    "back_right" : 43,
    "controller_type" : "talon"
}
pandemonium_ids = {
    "front_left" : 11,
    "back_left" : 21,
    "front_right" : 12,
    "back_right" : 22,
    "controller_type" : "spark_max"
}
# set this to change current robot
current_ids = pandemonium_ids


def curve(raw):
    # amount input is scaled by (keep at 0.5 to be safe)
    return 0.5*raw


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
        if current_ids["controller_type"] = "talon":
            self.frontLeftMotor = WPI_TalonFX(current_ids["front_left"])
            self.rearLeftMotor = WPI_TalonFX(current_ids["back_left"])
            self.frontRightMotor = WPI_TalonFX(current_ids["front_right"])
            self.rearRightMotor = WPI_TalonFX(current_ids["back_right"])
        else if current_ids["controller_type"] = "spark_max":
            self.frontLeftMotor = CANSparkMax(current_ids["front_left"])
            self.rearLeftMotor = CANSparkMax(current_ids["back_left"])
            self.frontRightMotor = CANSparkMax(current_ids["front_right"])
            self.rearRightMotor = CANSparkMax(current_ids["back_right"])

        self.left = wpilib.MotorControllerGroup(self.frontLeftMotor, self.rearLeftMotor)
        self.right = wpilib.MotorControllerGroup(
            self.frontRightMotor, self.rearRightMotor
        )
        self.right.setInverted(True)

        self.myRobot = DifferentialDrive(self.left, self.right)
        self.myRobot.setExpiration(0.1)

        # joysticks
        # self.leftStick = wpilib.Joystick(0)
        # self.rightStick = wpilib.Joystick(1)

        # xbox controller
        self.xbox = wpilib.XboxController(0)


    def teleopInit(self):
        """Executed at the start of teleop mode"""
        self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with arcade steering"""
        self.myRobot.arcadeDrive(-curve(self.xbox.getLeftY()), -curve(self.xbox.getLeftX()))


if __name__ == "__main__":
    wpilib.run(MyRobot)