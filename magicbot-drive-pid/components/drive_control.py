import wpimath.controller
import magicbot
from magicbot.state_machine import state
from magicbot import tunable
import navx

from components.drivetrain import Drivetrain
import util


class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain

    # variables to be injected
    navx: navx.AHRS

    """`tunable` numbers can be modified via Network Tables, and their value
    is stored locally on the robot so it will be kept even after the robot
    reboots. Also, this allows two different robots to have the same code
    and different configuration values.
    """
    turn_to_angle_kP = tunable(0.03)
    turn_to_angle_kI = tunable(0)
    turn_to_angle_kD = tunable(0)

    def setup(self):
        self.turn_to_angle_controller = wpimath.controller.PIDController(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )
        self.turn_to_angle_controller.enableContinuousInput(0, 360)

    def set_angle(self, angle: float):
        self.turn_to_angle_controller.setSetpoint(angle)

    def turn_to_angle(self) -> None:
        """Robot turns to set angle using a PID controller.

        This is a control function for the state machine, meaning that it,
        and not the state itself, must be called to engage it.
        """
        self.engage(initial_state="turning_to_angle")

    def turn_to_angle(self, angle: float) -> None:
        """Robot turns to set angle using a PID controller.

        This is a control function for the state machine, meaning that it,
        and not the state itself, must be called to engage it.
        """
        self.set_angle(angle)
        self.engage(initial_state="turning_to_angle")

    @state(first=True)
    def turning_to_angle(self):
        # update controller parameters with new data from networktables
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        measurement = self.navx.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        self.drivetrain.arcade_drive(0, -util.clamp(output, -0.3, 0.3))
