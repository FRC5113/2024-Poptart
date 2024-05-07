from enum import Enum

from wpilib import DoubleSolenoid, PneumaticsModuleType
from wpimath import kinematics
from magicbot import feedback, will_reset_to
from phoenix6.hardware.talon_fx import TalonFX
from phoenix6.configs.talon_fx_configs import TalonFXConfiguration
from phoenix6.controls.duty_cycle_out import DutyCycleOut
from phoenix6.signals import InvertedValue, NeutralModeValue


class OctoMode(Enum):
    DISABLED = 0
    DIFFERENTIAL_DRIVE = 1
    MECANUM_DRIVE = 2


class OctoModule:
    def __init__(
        self,
        motor_id: int,
        solenoid_forward_channel: int,
        solenoid_reverse_channel: int,
        isInverted: bool = False,
    ):
        self.motor = TalonFX(motor_id)
        config = TalonFXConfiguration()
        config.motor_output.neutral_mode = NeutralModeValue.COAST
        if isInverted:
            config.motor_output.inverted = InvertedValue.CLOCKWISE_POSITIVE
        else:
            config.motor_output.inverted = InvertedValue.COUNTER_CLOCKWISE_POSITIVE
        self.motor.configurator.apply(config)
        self.motor_speed = 0
        self.duty_cycle_out = DutyCycleOut(0, enable_foc=False)
        self.solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            solenoid_forward_channel,
            solenoid_reverse_channel,
        )
        self.mode = OctoMode.DISABLED
        # consider adding a buffer?

    def get_mode(self) -> OctoMode:
        return self.mode

    def get_solenoid_state(self) -> DoubleSolenoid.Value:
        return self.solenoid.get()

    def get_motor_speed(self) -> float:
        return self.motor_speed

    def get_duty_cycle(self) -> float:
        return self.duty_cycle_out.output

    def set_mode(self, mode: OctoMode) -> None:
        self.mode = mode

    def set_solenoid_state(self, state: DoubleSolenoid.Value) -> None:
        self.solenoid.set(state)

    def set_motor_speed(self, speed: float) -> None:
        self.motor_speed = speed

    def update(self) -> None:
        """Should be called every loop"""
        self.duty_cycle_out.output = self.motor_speed
        self.motor.set_control(self.duty_cycle_out)
        self.motor_speed = 0

        if (
            self.get_solenoid_state() != DoubleSolenoid.Value.kForward
            and self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE
        ):
            self.set_solenoid_state(DoubleSolenoid.Value.kForward)
        if (
            self.get_solenoid_state() != DoubleSolenoid.Value.kReverse
            and self.get_mode() == OctoMode.MECANUM_DRIVE
        ):
            self.set_solenoid_state(DoubleSolenoid.Value.kReverse)
        # try experimenting with using kOff to save power


class Drivetrain:
    # annotate motor and configuration instances
    front_left_module: OctoModule
    front_right_module: OctoModule
    back_left_module: OctoModule
    back_right_module: OctoModule
    differential_kinematics: kinematics.DifferentialDriveKinematics
    mecanum_kinematics: kinematics.MecanumDriveKinematics

    mode: OctoMode = OctoMode.DISABLED

    # values will reset to 0 after every time control loop runs
    x_speed: float = will_reset_to(0)
    y_speed: float = will_reset_to(0)
    z_rotation: float = will_reset_to(0)

    def setup(self):
        self.modules = [
            self.front_left_module,
            self.front_right_module,
            self.back_left_module,
            self.back_right_module,
        ]
        self.set_mode(OctoMode.MECANUM_DRIVE)

    def get_mode(self) -> OctoMode:
        return self.mode

    def set_mode(self, mode: OctoMode):
        self.mode = mode
        for module in self.modules:
            module.set_mode(self.mode)

    def arcade_drive(self, x_speed: float, z_rotation: float):
        assert -1.0 < x_speed < 1.0, f"Improper x_speed: {x_speed}"
        assert -1.0 < z_rotation < 1.0, f"Improper z_rotation: {z_rotation}"
        self.x_speed = x_speed
        self.y_speed = 0
        self.z_rotation = z_rotation

    def cartesian_drive(self, x_speed: float, y_speed: float, z_rotation: float):
        # consider using field-oriented drive
        assert -1.0 < x_speed < 1.0, f"Improper x_speed: {x_speed}"
        assert -1.0 < y_speed < 1.0, f"Improper y_speed: {x_speed}"
        assert -1.0 < z_rotation < 1.0, f"Improper z_rotation: {z_rotation}"
        assert (
            self.get_mode() == OctoMode.MECANUM_DRIVE
        ), "Cannot use cartesian_drive in tank mode"
        self.x_speed = x_speed
        self.y_speed = y_speed
        self.z_rotation = z_rotation

    def execute(self):
        for module in self.modules:
            module.update()

        chassis_speeds = kinematics.ChassisSpeeds(
            self.x_speed, self.y_speed, self.z_rotation
        )
        # technically the wheel speeds should be scaled to convert from m/s to output % but whatever
        if self.get_mode() == OctoMode.MECANUM_DRIVE:
            wheel_speeds = self.mecanum_kinematics.toWheelSpeeds(chassis_speeds)
            self.front_left_module.set_motor_speed(wheel_speeds.frontLeft)
            self.front_right_module.set_motor_speed(wheel_speeds.frontRight)
            self.back_left_module.set_motor_speed(wheel_speeds.rearLeft)
            self.back_right_module.set_motor_speed(wheel_speeds.rearRight)
        if self.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
            wheel_speeds = self.differential_kinematics.toWheelSpeeds(chassis_speeds)
            self.front_left_module.set_motor_speed(wheel_speeds.left)
            self.front_right_module.set_motor_speed(wheel_speeds.right)
            self.back_left_module.set_motor_speed(wheel_speeds.left)
            self.back_right_module.set_motor_speed(wheel_speeds.right)

    @feedback
    def get_nt_mode(self) -> int:
        return self.get_mode().value

    @feedback
    def get_fl_speed(self) -> float:
        return self.front_left_module.get_motor_speed()

    @feedback
    def get_fr_speed(self) -> float:
        return self.front_right_module.get_motor_speed()

    @feedback
    def get_bl_speed(self) -> float:
        return self.back_left_module.get_motor_speed()

    @feedback
    def get_br_speed(self) -> float:
        return self.back_right_module.get_motor_speed()
