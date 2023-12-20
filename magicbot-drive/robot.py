#!/usr/bin/env python3

import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        self.front_left_drive_motor = CANSparkMax(11,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.front_right_drive_motor = CANSparkMax(12,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.back_left_drive_motor = CANSparkMax(21,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.back_right_drive_motor = CANSparkMax(22,CANSparkMaxLowLevel.MotorType.kBrushless)

        self.joystick = wpilib.Joystick(0)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        try:
            self.drivetrain.arcade_drive(-curve(self.joystick.getY()), -curve(self.joystick.getX()))
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(MyRobot)