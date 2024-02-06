import wpimath.controller
import magicbot
from magicbot.state_machine import state, default_state
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
        self.turn_to_angle_controller.setSetpoint(angle)

    # control functions called from the main loop (must be continually called)
    def turn_to_angle(self, angle: float = None):
        if angle is not None:
            self.set_angle(angle)
        self.engage(initial_state="turning_to_angle")

    def set_drive_from_tag(self, distance: float):
        self.drive_from_tag_setpoint = distance

    def drive_from_tag(self):
        # drives set distances away from tag
        if self.vision.hasTargets():
            self.engage(initial_state="driving_from_tag")

    def drive_from_tag(self, distance: float):
        self.set_drive_from_tag(distance)
        self.drive_from_tag()

    def turn_to_tag(self):
        # turns to face AprilTag
        if self.vision.hasTargets():
            self.turn_to_angle(self.gyro.getAngle() + self.vision.getHeading())

    def follow_tag(self):
        # faces tag and drives set distance away from it
        if self.vision.hasTargets():
            self.engage(initial_state="following_tag")

    def follow_tag(self, distance: float):
        self.set_drive_from_tag(distance)
        self.follow_tag()

    def tag_control(self):
        # drive forward when seeing one AprilTag, drive backward for another
        if self.vision.getId() == 1:
            self.engage(initial_state="driving_forwards")
        elif self.vision.getId() == 2:
            self.engage(initial_state="driving_backwards")

    @state(first=True)
    def idle(self):
        # first state intentionally does nothing to avoid accidents
        self.drivetrain.stop()

    @state()
    def driving_forwards(self):
        self.drivetrain.arcade_drive(0.3, 0)

    @state
    def driving_backwards(self):
        self.drivetrain.arcade_drive(-0.3, 0)

    @state
    def turning_to_angle(self):
        # as long as turn_to_angle() is called, this state will execute

        # update controller parameters with new data from networktables
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        measurement = self.gyro.getAngle()
        output = self.turn_to_angle_controller.calculate(measurement)
        self.drivetrain.arcade_drive(0, -util.clamp(output, -0.3, 0.3))

    @state
    def driving_from_tag(self):
        measurement = self.vision.getX()
        error = self.drive_from_tag_setpoint - measurement
        output = error * self.drive_from_tag_kP
        self.drivetrain.arcade_drive(util.clamp(output, -0.3, 0.3), 0)

    @state
    def following_tag(self):
        self.turn_to_angle_controller.setPID(
            self.turn_to_angle_kP, self.turn_to_angle_kI, self.turn_to_angle_kD
        )

        angle_measurement = self.gyro.getAngle()
        turn_output = self.turn_to_angle_controller.calculate(angle_measurement)

        distance_measurement = self.vision.getX()
        distance_error = self.drive_from_tag_setpoint - distance_measurement
        forward_output = distance_error * self.drive_from_tag_kP

        self.drivetrain.arcade_drive(
            util.clamp(forward_output, -0.3, 0.3), -util.clamp(turn_output, -0.3, 0.3)
        )
