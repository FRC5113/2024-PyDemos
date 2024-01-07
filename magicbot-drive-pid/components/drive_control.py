import wpimath.controller
import magicbot
from magicbot.state_machine import state
import navx

from components.drivetrain import Drivetrain
import config


class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain

    # variables to be injected
    navx: navx.AHRS

    def __init__(self):
        # initialize controllers
        cfg = config.TurnToAngle_cfg
        self.turn_to_angle_controller = wpimath.controller.PIDController(
            cfg.kP, cfg.kI, cfg.kD
        )

    def turn_to_angle(self, angle: float):
        # control function called from the main loop
        self.turn_to_angle_controller.setSetpoint(angle)
        self.engage(initial_state="turning_to_angle")

    @state
    def turning_to_angle(self):
        # as long as turn_to_angle() is called, this state will execute
        measurement = self.navx.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        self.drivetrain.arcade_drive(0, output)
