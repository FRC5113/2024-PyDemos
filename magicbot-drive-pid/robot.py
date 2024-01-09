#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from magicbot import MagicRobot, feedback
import navx

from components.drive_control import DriveControl
from components.drivetrain import Drivetrain
from config import pandemonium_cfg


drivetrain_cfg = pandemonium_cfg


def curve(a):
    """Adjust raw input value for better control of drivetrain"""
    return 0.3 * a


class MyRobot(MagicRobot):
    #
    # Define components here (high level before low level)
    #

    drive_control: DriveControl

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        if drivetrain_cfg.controller_type == "SPARK_MAX":
            self.drivetrain_front_left_motor = CANSparkMax(
                drivetrain_cfg.front_left_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.drivetrain_front_right_motor = CANSparkMax(
                drivetrain_cfg.front_right_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.drivetrain_back_left_motor = CANSparkMax(
                drivetrain_cfg.back_left_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
            self.drivetrain_back_right_motor = CANSparkMax(
                drivetrain_cfg.back_right_id, CANSparkMaxLowLevel.MotorType.kBrushless
            )
        else:
            raise Exception(
                f"Improper controller type in drivetrain_cfg: {drivetrain_cfg.controller_type}"
            )

        self.navx = navx.AHRS.create_spi()
        self.joystick = wpilib.Joystick(0)

    def teleopInit(self):
        """Called right before teleop control loop starts"""
        self.drivetrain.drive.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # state machine will only execute when button is held for safety reasons
            if self.joystick.getTrigger():
                self.drive_control.turn_to_angle(180)
            else:
                self.drivetrain.arcade_drive(
                    curve(self.joystick.getY()), -curve(self.joystick.getX())
                )

    @feedback
    def get_angle(self):
        return self.navx.getAngle()


if __name__ == "__main__":
    wpilib.run(MyRobot)
