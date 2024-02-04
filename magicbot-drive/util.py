from wpilib.interfaces import MotorController
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.controls.voltage_out import VoltageOut
from phoenix6.signals import InvertedValue, NeutralModeValue


class WPI_TalonFX(TalonFX, MotorController):
    """Wrapper for the phoenix6 TalonFX that implements
    the wpilib MotorController interface, making it possible
    to use TalonFX controllers in, for example, MotorControllerGroup
    and DifferentialDrive
    """

    def __init__(self, id: int, canbus: str = "", enable_foc: bool = False):
        TalonFX.__init__(id, canbus=canbus)
        self.config = TalonFXConfiguration()
        self.duty_cycle_out = DutyCycleOut(0, enable_foc=enable_foc)
        self.voltage_out = VoltageOut(0, enable_foc=enable_foc)
        self.is_disabled = False

    def disable(self):
        self.stopMotor()
        self.is_disabled = True

    def get(self) -> float:
        return self.duty_cycle_out.output

    def getInverted(self) -> bool:
        return (
            self.config.motor_output.inverted
            == InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        )

    def set(self, speed: float):
        if not self.is_disabled:
            self.duty_cycle_out.output = speed
            self.set_control(self.duty_cycle_out)

    def setIdleMode(self, mode: NeutralModeValue):
        """Set the idle mode setting

        Arguments:
        mode -- Idle mode (coast or brake)
        """
        self.config.motor_output.neutral_mode = mode
        self.configurator.apply(self.config)

    def setInverted(self, isInverted: bool):
        if isInverted:
            self.config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            self.config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.configurator.apply(self.config)

    def setVoltage(self, volts: float):
        if not self.is_disabled:
            self.voltage_out.output = volts
            self.set_control(self.voltage_out)

    def stopMotor(self):
        self.set(0)
