#!/usr/bin/env python3
import wpilib
from rev import CANSparkMax, CANSparkMaxLowLevel
from ctre import WPI_TalonFX
from magicbot import MagicRobot, feedback
import navx
from photonvision import PhotonCamera

from components.drive_control import DriveControl
from components.drivetrain import Drivetrain
from components.vision import Vision
from config import *


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
    vision: Vision

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
        elif drivetrain_cfg.controller_type == "TALON":
            self.drivetrain_front_left_motor = WPI_TalonFX(drivetrain_cfg.front_left_id)
            self.drivetrain_front_right_motor = WPI_TalonFX(drivetrain_cfg.front_right_id)
            self.drivetrain_back_left_motor = WPI_TalonFX(drivetrain_cfg.back_left_id)
            self.drivetrain_back_right_motor = WPI_TalonFX(drivetrain_cfg.back_right_id)
        else:
            raise Exception(
                f"Improper controller type in drivetrain_cfg: {drivetrain_cfg.controller_type}"
            )

        self.camera = PhotonCamera("Global_Shutter_Camera")
        self.vision_filter_window = 10

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
    def get_angle(self) -> float:
        return self.navx.getAngle()

    @feedback
    def get_id(self) -> int:
        return self.vision.getId()


if __name__ == "__main__":
    wpilib.run(MyRobot)
