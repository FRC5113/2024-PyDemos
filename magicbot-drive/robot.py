#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
from config import pandemonium_cfg


def curve(a):
    """Adjust raw input value for better control of drivetrain"""
    return a


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: Drivetrain
    drivetrain_cfg = pandemonium_cfg

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.joystick = wpilib.Joystick(0)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        try:
            self.drivetrain.arcade_drive(
                curve(self.joystick.getY()), -curve(self.joystick.getX())
            )
        except:
            self.onException()


if __name__ == "__main__":
    wpilib.run(MyRobot)
