import wpimath.controller
import magicbot
from magicbot.state_machine import state
from magicbot import tunable
import navx

from components.drivetrain import Drivetrain


def clamp(value: float, min_value: float, max_value: float):
    return max(min(value, max_value), min_value)


class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain

    # variables to be injected
    navx: navx.AHRS

    turn_to_angle_kP = tunable(0)
    turn_to_angle_kI = tunable(0)
    turn_to_angle_kD = tunable(0)

    def setup(self):
        self.turn_to_angle_controller = wpimath.controller.PIDController(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )
        self.turn_to_angle_controller.enableContinuousInput(0, 360)

    def set_angle(self, angle: float):
        self.turn_to_angle_controller.setSetpoint(angle)

    def turn_to_angle(self):
        self.engage(initial_state="turning_to_angle")

    def turn_to_angle(self, angle: float):
        # control function called from the main loop
        self.set_angle(angle)
        self.engage(initial_state="turning_to_angle")

    @state(first=True)
    def turning_to_angle(self):
        # as long as turn_to_angle() is called, this state will execute

        # update controller parameters with new data from networktables
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        measurement = self.navx.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        self.drivetrain.arcade_drive(0, clamp(output, -0.3, 0.3))
