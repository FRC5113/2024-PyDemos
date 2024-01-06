#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import MagicRobot

from components.drivetrain import SparkMaxDrivetrain
from config import pandemonium_cfg


drivetrain_cfg = pandemonium_cfg

def curve(a):
    """Adjust raw input value for better control of drivetrain"""
    return a


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: SparkMaxDrivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_front_left_motor = CANSparkMax(drivetrain_cfg.front_left_id,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.drivetrain_front_right_motor = CANSparkMax(drivetrain_cfg.front_right_id,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.drivetrain_back_left_motor = CANSparkMax(drivetrain_cfg.back_left_id,CANSparkMaxLowLevel.MotorType.kBrushless)
        self.drivetrain_back_right_motor = CANSparkMax(drivetrain_cfg.back_right_id,CANSparkMaxLowLevel.MotorType.kBrushless)

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
