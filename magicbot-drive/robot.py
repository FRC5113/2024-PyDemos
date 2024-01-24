#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot

from components.drivetrain import Drivetrain
from config import pandemonium_cfg


drivetrain_cfg = pandemonium_cfg


def curve(a):
    """Adjust raw input value for better control of drivetrain"""
    return a


class MyRobot(MagicRobot):
    #
    # Define components here
    #

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        if drivetrain_cfg.controller_type == "SPARK_MAX":
            self.drivetrain_front_left_motor = CANSparkMax(
                drivetrain_cfg.front_left_id, CANSparkLowLevel.MotorType.kBrushless
            )
            self.drivetrain_front_right_motor = CANSparkMax(
                drivetrain_cfg.front_right_id, CANSparkLowLevel.MotorType.kBrushless
            )
            self.drivetrain_back_left_motor = CANSparkMax(
                drivetrain_cfg.back_left_id, CANSparkLowLevel.MotorType.kBrushless
            )
            self.drivetrain_back_right_motor = CANSparkMax(
                drivetrain_cfg.back_right_id, CANSparkLowLevel.MotorType.kBrushless
            )
        else:
            raise Exception(
                f"Improper controller type in drivetrain_cfg: {drivetrain_cfg.controller_type}"
            )

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
