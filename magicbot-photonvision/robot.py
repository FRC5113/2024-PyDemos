#!/usr/bin/env python3
"""This is a demo program showcasing the use of the photonlibpy library
to use Apriltags in a magicbot project. In this case, the robot can be made to
identify, turn to, or follow Apriltags.
"""

import wpilib
from rev import CANSparkMax, CANSparkLowLevel
import phoenix5
from magicbot import MagicRobot, feedback
import navx
from photonlibpy.photonCamera import PhotonCamera

from components.drive_control import DriveControl
from components.drivetrain import Drivetrain
from components.vision import Vision
import config
import util


drivetrain_cfg = config.pancake_cfg


class MyRobot(MagicRobot):
    #
    # Define components here (high level before low level)
    #

    drive_control: DriveControl

    drivetrain: Drivetrain
    vision: Vision

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_controller_type = drivetrain_cfg.controller_type
        if drivetrain_cfg.controller_type == config.ControllerType.SPARK_MAX:
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
        elif drivetrain_cfg.controller_type == config.ControllerType.TALON_FX:
            self.drivetrain_front_left_motor = util.WPI_TalonFX(
                drivetrain_cfg.front_left_id
            )
            self.drivetrain_front_right_motor = util.WPI_TalonFX(
                drivetrain_cfg.front_right_id
            )
            self.drivetrain_back_left_motor = util.WPI_TalonFX(
                drivetrain_cfg.back_left_id
            )
            self.drivetrain_back_right_motor = util.WPI_TalonFX(
                drivetrain_cfg.back_right_id
            )
        elif drivetrain_cfg.controller_type == config.ControllerType.TALON_SRX:
            self.drivetrain_front_left_motor = phoenix5.WPI_TalonSRX(
                drivetrain_cfg.front_left_id
            )
            self.drivetrain_front_right_motor = phoenix5.WPI_TalonSRX(
                drivetrain_cfg.front_right_id
            )
            self.drivetrain_back_left_motor = phoenix5.WPI_TalonSRX(
                drivetrain_cfg.back_left_id
            )
            self.drivetrain_back_right_motor = phoenix5.WPI_TalonSRX(
                drivetrain_cfg.back_right_id
            )
        else:
            raise Exception(
                f"Improper controller type in `drivetrain_cfg`: {drivetrain_cfg.controller_type}"
            )

        self.camera = PhotonCamera("Global_Shutter_Camera")
        self.vision_filter_window = 10

        self.gyro = navx.AHRS.create_spi()
        self.joystick = wpilib.Joystick(0)
        self.curve = util.linear_curve(scalar=0.5, deadband=0.1, max_mag=1)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # state machine will only execute when button is held for safety reasons
            if self.joystick.getTrigger():
                self.drive_control.follow_tag()
            else:
                self.drivetrain.arcade_drive(
                    self.curve(self.joystick.getY()), -self.curve(self.joystick.getX())
                )

    @feedback
    def get_angle(self) -> float:
        return self.gyro.getAngle()

    @feedback
    def get_id(self) -> int:
        id = self.vision.getId()
        if id is not None:
            return id
        return -1

    @feedback
    def get_x(self) -> int:
        if self.vision.hasTargets():
            return self.vision.getX()
        return 0

    @feedback
    def get_y(self) -> int:
        if self.vision.hasTargets():
            return self.vision.getY()
        return 0

    @feedback
    def get_z(self) -> int:
        if self.vision.hasTargets():
            return self.vision.getZ()
        return 0

    @feedback
    def get_heading(self) -> int:
        if self.vision.hasTargets():
            return self.vision.getHeading()
        return 0


if __name__ == "__main__":
    wpilib.run(MyRobot)
