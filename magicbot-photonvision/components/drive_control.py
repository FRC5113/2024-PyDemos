import wpimath.controller
import magicbot
from magicbot.state_machine import state
from magicbot import tunable
import navx

from components.drivetrain import Drivetrain
from components.vision import Vision
import util


class DriveControl(magicbot.StateMachine):
    # other components
    drivetrain: Drivetrain
    vision: Vision

    # variables to be injected
    gyro: navx.AHRS

    turn_to_angle_kP = tunable(0.03)
    turn_to_angle_kI = tunable(0)
    turn_to_angle_kD = tunable(0)

    drive_from_tag_kP = tunable(0)
    drive_from_tag_setpoint = 0

    def setup(self):
        # setup() required because tunables need to be fetched
        self.turn_to_angle_controller = wpimath.controller.PIDController(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )
        self.turn_to_angle_controller.enableContinuousInput(0, 360)

    def set_angle(self, angle: float):
        """Changes the `turn_to_angle` PID controller's setpoint"""
        self.turn_to_angle_controller.setSetpoint(angle)

    # control functions called from the main loop (must be continually called)
    def turn_to_angle(self, angle: float = None):
        """Call this function to engage the `turning_to_angle` state.
        If no value is passed to `angle`, it will turn to whatever
        angle has been last set.
        """
        if angle is not None:
            self.set_angle(angle)
        self.engage(initial_state="turning_to_angle")

    def set_drive_from_tag(self, distance: float):
        """Changes the `drive_from_tag` P controller's setpoint"""
        self.drive_from_tag_setpoint = distance

    def drive_from_tag(self, distance: float = None):
        """Call this function to engage the `driving_from_tag` state.
        If no value is passed to `distance`, it will use whatever
        distance has been last set.
        """
        if distance is not None:
            self.set_drive_from_tag(distance)
        if self.vision.hasTargets():
            self.engage(initial_state="driving_from_tag")

    def turn_to_tag(self):
        """Changes the `turn_to_angle` setpoint to one such that the robot
        would face an AprilTag
        """
        if self.vision.hasTargets():
            self.turn_to_angle(self.gyro.getAngle() + self.vision.getHeading())

    def follow_tag(self, distance: float = None):
        """Call this function to engage the `following_tag` state.
        If no value is passed to `distance`, it will use whatever
        distance has been last set.
        """
        if distance is not None:
            self.set_drive_from_tag(distance)
        if self.vision.hasTargets():
            self.engage(initial_state="following_tag")

    def tag_control(self):
        """Call this function to engage the `driving_forwards` or
        `driving_backwards` state based on if a tag of a certain fiducial ID
        is detected (IDs 1 and 2 respectively).
        """
        """Note the use of `force=True` which is needed to force the state
        machine to switch states upon seeing a different ID
        """
        if self.vision.getId() == 1:
            self.engage(initial_state="driving_forwards", force=True)
        elif self.vision.getId() == 2:
            self.engage(initial_state="driving_backwards", force=True)
        else:
            self.done()

    @state(first=True)
    def idle(self):
        """First state -- does nothing to avoid accidents if `engage()`
        is called without an initial state.
        """
        self.drivetrain.stop()

    @state()
    def driving_forwards(self):
        """State in which robot drives forwards at 30% speed"""
        self.drivetrain.arcade_drive(0.3, 0)

    @state
    def driving_backwards(self):
        """State in which robot drives backwards at 30% speed"""
        self.drivetrain.arcade_drive(-0.3, 0)

    @state
    def turning_to_angle(self):
        """State in which robot uses a PID controller to turn to a certain
        angle using sensor data from the gyroscope.
        """
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        measurement = self.gyro.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        """Here (and elsewhere) the output is negated because a positive turn
        value in `arcade_drive()` corresponds with a decrease in angle.
        This could also be fixed with negative PID values, but this is not
        recommended.
        """
        self.drivetrain.arcade_drive(0, util.clamp(-output, -0.3, 0.3))

    @state
    def driving_from_tag(self):
        """State in which robot drives forward or backward so that it is
        a set distance away from a detected Apriltag
        """
        measurement = self.vision.getX()
        error = self.drive_from_tag_setpoint - measurement
        output = error * self.drive_from_tag_kP
        """Here the output is negated because a positive error means that
        the setpoint is greater than the measurement, which means the robot
        is too close. If so, it must drive backward (ie with negative velocity)
        """
        self.drivetrain.arcade_drive(util.clamp(-output, -0.3, 0.3), 0)

    @state
    def following_tag(self):
        """State in which robot drives so that it both is a set distance
        away from a detected Apriltag and faces the Apriltag. In theory,
        both of these effects combined would make the robot "follow" a tag.
        """
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        angle_measurement = self.gyro.getAngle()
        turn_output = self.turn_to_angle_controller.calculate(angle_measurement)

        distance_measurement = self.vision.getX()
        distance_error = self.drive_from_tag_setpoint - distance_measurement
        forward_output = distance_error * self.drive_from_tag_kP

        self.drivetrain.arcade_drive(
            util.clamp(-forward_output, -0.3, 0.3), util.clamp(-turn_output, -0.3, 0.3)
        )
