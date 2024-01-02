#!/usr/bin/env python3

from collections import namedtuple

import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain

# Configuration objects will be injected into component classes
DrivetrainConfig = namedtuple(
    "DrivetrainConfig",
    "front_left_id front_right_id back_left_id back_right_id controller_type",
)
pandemonium_cfg = DrivetrainConfig(
    front_left_id=11,
    front_right_id=12,
    back_left_id=21,
    back_right_id=22,
    controller_type="SPARKMAX",
)


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
