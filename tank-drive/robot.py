#!/usr/bin/env python3
"""
    This is a demo program showing the use of the RobotDrive class,
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


def curve(raw):
    # amount input is scaled by (keep at 0.5 to be safe)
    return 0.5 * raw


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        """Robot initialization function"""

        # object that handles basic drive operations
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
            rightConfig.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
            self.frontRightMotor.configurator.apply(rightConfig)
            self.rearRightMotor.configurator.apply(rightConfig)

            self.rearLeftMotor.set_control(Follower(current_ids["front_left"], False))
            self.rearRightMotor.set_control(Follower(current_ids["front_right"], False))

        # joysticks
        # self.leftStick = wpilib.Joystick(0)
        # self.rightStick = wpilib.Joystick(1)

        # xbox controller
        self.xbox = wpilib.XboxController(0)

    def teleopInit(self):
        """Executed at the start of teleop mode"""
        if current_ids["controller_type"] == "spark_max":
            self.myRobot.setSafetyEnabled(True)
        # elif current_ids["controller_type"] == "talon":
        #     self.frontLeftMotor.setSafetyEnabled(True)
        #     self.frontRightMotor.setSafetyEnabled(True)

    def teleopPeriodic(self):
        """Runs the motors with tank steering"""
        forward = curve(self.xbox.getLeftY())
        turn = -curve(self.xbox.getLeftX())
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
