#!/usr/bin/env python3
"""
    This is a demo program showing the use of the TimedRobot class,
    specifically it contains the code necessary to operate a robot with
    tank drive.
"""

import wpilib
from wpilib.drive import DifferentialDrive
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.follower import Follower
from phoenix6.signals import InvertedValue
from rev import CANSparkMax
from rev import CANSparkLowLevel

import util

# CAN ids of three of our robots
pancake_ids = {
    "front_left": 8,
    "back_left": 7,
    "front_right": 9,
    "back_right": 11,
    "controller_type": "talon",
}
perry_ids = {
    "front_left": 41,
    "back_left": 42,
    "front_right": 44,
    "back_right": 43,
    "controller_type": "talon",
}
pandemonium_ids = {
    "front_left": 11,
    "back_left": 21,
    "front_right": 12,
    "back_right": 22,
    "controller_type": "spark_max",
}
# set this to change current robot
current_ids = pancake_ids


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function

        Here, two different ways of achieving differential drive are showcased.
        In the first (with the Spark Max controller), the motor controllers are
        combined into two groups and then combined into one DifferentialDrive
        object. This is the preferred way of doing this, but some motor
        controllers (such as any TalonFX controller with phoenix6) do not
        implement the MotorController interface from wpilib and therefore
        cannot be used in MotorControllerGroup objects. Instead, two motors
        are configured to follow two others and speeds are set for each of the
        leader controllers.

        As with most drivetrains, one side is inverted here because the motors
        on one side are rotated 180 degrees compared to the other, so the
        definition of "forward" changes based on the side of the robot that
        the motor is on.
        """

        if current_ids["controller_type"] == "spark_max":
            self.frontLeftMotor = CANSparkMax(
                current_ids["front_left"], CANSparkLowLevel.MotorType.kBrushless
            )
            self.rearLeftMotor = CANSparkMax(
                current_ids["back_left"], CANSparkLowLevel.MotorType.kBrushless
            )
            self.frontRightMotor = CANSparkMax(
                current_ids["front_right"], CANSparkLowLevel.MotorType.kBrushless
            )
            self.rearRightMotor = CANSparkMax(
                current_ids["back_right"], CANSparkLowLevel.MotorType.kBrushless
            )

            self.left = wpilib.MotorControllerGroup(
                self.frontLeftMotor, self.rearLeftMotor
            )
            self.right = wpilib.MotorControllerGroup(
                self.frontRightMotor, self.rearRightMotor
            )
            self.right.setInverted(True)
            self.myRobot = DifferentialDrive(self.left, self.right)
            self.myRobot.setExpiration(0.1)
        elif current_ids["controller_type"] == "talon":
            self.frontLeftMotor = TalonFX(current_ids["front_left"])
            self.rearLeftMotor = TalonFX(current_ids["back_left"])
            self.frontRightMotor = TalonFX(current_ids["front_right"])
            self.rearRightMotor = TalonFX(current_ids["back_right"])

            self.leftOut = DutyCycleOut(0, enable_foc=False)
            self.rightOut = DutyCycleOut(0, enable_foc=False)

            rightConfig = TalonFXConfiguration()
            # default is counter clockwise
            rightConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
            self.frontRightMotor.configurator.apply(rightConfig)
            self.rearRightMotor.configurator.apply(rightConfig)

            self.rearLeftMotor.set_control(Follower(current_ids["front_left"], False))
            self.rearRightMotor.set_control(Follower(current_ids["front_right"], False))

        self.xbox = wpilib.XboxController(0)
        self.curve = util.linear_curve(scalar=0.5, deadband=0.1, max_mag=1)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        if current_ids["controller_type"] == "spark_max":
            self.myRobot.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering

        If a DifferentialDrive object has been created, forward and turn
        inputs can be supplied into the object and it will handle setting
        the motor speeds. If not, the speeds must first be calculated by
        an inverse kinematics function and then manually fed into the
        leader motors.
        """
        forward = self.curve(self.xbox.getLeftY())
        turn = -self.curve(self.xbox.getLeftX())
        if current_ids["controller_type"] == "spark_max":
            self.myRobot.arcadeDrive(forward, turn)
        elif current_ids["controller_type"] == "talon":
            speeds = DifferentialDrive.arcadeDriveIK(forward, turn)
            self.leftOut.output = speeds.left
            self.rightOut.output = speeds.right
            self.frontLeftMotor.set_control(self.leftOut)
            self.frontRightMotor.set_control(self.rightOut)


if __name__ == "__main__":
    wpilib.run(MyRobot)
