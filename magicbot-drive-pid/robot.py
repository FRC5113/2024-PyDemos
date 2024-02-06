#!/usr/bin/env python3
"""This is a demo program showcasing the use of PID control in a
magicbot state machine. Specifically, this program contains a PID controller
that allows the robot to turn to a certain angle.
"""

import wpilib
from rev import CANSparkMax, CANSparkLowLevel
from magicbot import MagicRobot, feedback
import navx

from components.drive_control import DriveControl
from components.drivetrain import Drivetrain
import config
import util
from util import WPI_TalonFX


drivetrain_cfg = config.pancake_cfg


class MyRobot(MagicRobot):
    #
    # Define components here (high level before low level)
    #

    drive_control: DriveControl

    drivetrain: Drivetrain

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
            self.drivetrain_front_left_motor = WPI_TalonFX(
                drivetrain_cfg.front_left_id
            )
            self.drivetrain_front_right_motor = WPI_TalonFX(
                drivetrain_cfg.front_right_id
            )
            self.drivetrain_back_left_motor = WPI_TalonFX(
                drivetrain_cfg.back_left_id
            )
            self.drivetrain_back_right_motor = WPI_TalonFX(
                drivetrain_cfg.back_right_id
            )
        else:
            raise Exception(
                f"Improper controller type in `drivetrain_cfg`: {drivetrain_cfg.controller_type}"
            )

        self.navx = navx.AHRS.create_spi()
        self.joystick = wpilib.Joystick(0)
        self.curve = util.linear_curve(scalar=0.5, deadband=0.1, max_mag=1)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # state machine will only execute when button is held for safety reasons
            if self.joystick.getTrigger():
                self.drive_control.turn_to_angle(180)
            else:
                self.drivetrain.arcade_drive(
                    self.curve(self.joystick.getY()), -self.curve(self.joystick.getX())
                )

    @feedback
    def get_angle(self):
        return self.navx.getAngle()


if __name__ == "__main__":
    wpilib.run(MyRobot)
