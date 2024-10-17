from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
import wpimath.units as units

from phoenix6 import hardware, controls, configs, signals

class swerve_module_constants:
    WHEEL_CIR = units.inchesToMeters(3.94) * 3.1415926
    DRIVE_RATIO = (5.36 / 1.0)
    ANGLE_RATIO = (150.0 / 7.0 / 1.0)

    THEORETICAL_MAX_SPEED = 6000 / 60 / DRIVE_RATIO * WHEEL_CIR; # 5.13 mps
    MAX_SPEED = THEORETICAL_MAX_SPEED * 0.85; # MPS

    MAX_ANG_SPEED = 8; # MPS^2

    def __init__(self, cancoder_id, angle_id, drive_id, offset_deg):
        self._encoderID = cancoder_id
        self._angleID = angle_id
        self._driveID = drive_id
        self._offset = Rotation2d.fromDegrees(offset_deg)

class swerve_module_configs:
    cancoder = configs.CANcoderConfiguration()
    cancoder.magnet_sensor.sensor_direction = signals.SensorDirectionValue.COUNTER_CLOCKWISE_POSITIVE
    cancoder.magnet_sensor.absolute_sensor_range = signals.AbsoluteSensorRangeValue.SIGNED_PLUS_MINUS_HALF

    angle = configs.TalonFXConfiguration()
    angle.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
    angle.motor_output.neutral_mode = signals.NeutralModeValue.COAST
    angle.feedback.sensor_to_mechanism_ratio = swerve_module_constants.ANGLE_RATIO
    angle.closed_loop_general.continuous_wrap = True
    angle.current_limits.stator_current_limit_enable = True
    angle.current_limits.stator_current_limit = 40
    angle.current_limits.supply_current_limit_enable = True
    angle.current_limits.supply_current_limit = 25
    angle.current_limits.supply_current_threshold = 40
    angle.current_limits.supply_time_threshold = 0.1
    angle.voltage.peak_forward_voltage = 12.
    angle.voltage.peak_reverse_voltage = -12.
    angle.slot0.k_p = 128.
    angle.slot0.k_i = 0.
    angle.slot0.k_d = 0.
    angle.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0
    angle.closed_loop_ramps.voltage_closed_loop_ramp_period = 0
    angle.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0
    angle.open_loop_ramps.voltage_open_loop_ramp_period = 0


    drive = configs.TalonFXConfiguration()
    drive.motor_output.inverted = signals.InvertedValue.CLOCKWISE_POSITIVE
    drive.motor_output.neutral_mode = signals.NeutralModeValue.BRAKE
    drive.feedback.sensor_to_mechanism_ratio = swerve_module_constants.DRIVE_RATIO
    drive.current_limits.stator_current_limit_enable = True
    drive.current_limits.stator_current_limit = 60
    drive.current_limits.supply_current_limit_enable = True
    drive.current_limits.supply_current_limit = 25
    drive.current_limits.supply_current_threshold = 60
    drive.current_limits.supply_time_threshold = 0.1
    drive.voltage.peak_forward_voltage = 12.
    drive.voltage.peak_reverse_voltage = -12.
    drive.slot0.k_p = 2.
    drive.slot0.k_i = 0.
    drive.slot0.k_d = 0.
    drive.closed_loop_ramps.duty_cycle_closed_loop_ramp_period = 0.02
    drive.closed_loop_ramps.voltage_closed_loop_ramp_period = 0.02
    drive.open_loop_ramps.duty_cycle_open_loop_ramp_period = 0.1
    drive.open_loop_ramps.voltage_open_loop_ramp_period = 0.1

class swerve_module:
    def __init__(self, mod_id, mod_consts):
        self._id = mod_id
        self._offset = mod_consts._offset

        self._angleEncoder = hardware.CANcoder(mod_consts._encoderID, "rio")
        self._angleEncoder.configurator.apply(swerve_module_configs.cancoder)

        self._angleMotor = hardware.TalonFX(mod_consts._angleID, "rio")
        self._angleMotor.configurator.apply(swerve_module_configs.angle)

        self.resetToAbsolute();

        self._driveMotor = hardware.TalonFX(mod_consts._driveID, "rio");
        self._driveMotor.configurator.apply(swerve_module_configs.drive)

        self._lastAngle = self.getState().angle;
    
        self._feedforward = SimpleMotorFeedforwardMeters(0.667 / 12,2.44 / 12,0.27 / 12)

        self._driveOutput = controls.DutyCycleOut(0)
        self._driveVelocity = controls.VelocityVoltage(0)
        self._anglePosition = controls.PositionVoltage(0)

        self._angleEncoder.get_absolute_position().set_update_frequency(100)
        self._angleMotor.get_position().set_update_frequency(100)
        self._driveMotor.get_position().set_update_frequency(100)
        self._driveMotor.get_velocity().set_update_frequency(100)

        self._angleEncoder.optimize_bus_utilization_for_all(self._angleEncoder, self._angleMotor, self._driveMotor)

    def resetToAbsolute(self) -> None:
        absolutePosition = self.getEncoderAngle() - self._offset;
        self._angleMotor.set_position(absolutePosition.degrees()/360);

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(
            self._driveMotor.get_velocity().value_as_double * swerve_module_constants.WHEEL_CIR, 
            Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double))
    
    def setDesiredState(self, desiredState, openLoop) -> None:
        desiredState = SwerveModuleState.optimize(desiredState, Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double))
        angle = self._lastAngle if abs(desiredState.speed) <= (swerve_module_constants.MAX_SPEED * 0.01) else desiredState.angle;
        self._angleMotor.set_control(self._anglePosition.with_position(angle.degrees()/360));
        self.setSpeed(desiredState, openLoop);
        self._lastAngle = angle;

    def setSpeed(self, desiredState, openLoop) -> None:
        ratio = (desiredState.angle - Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double)).cos(); 
        if (openLoop):
            self._driveMotor.set_control(self._driveOutput.with_output(desiredState.speed / swerve_module_constants.MAX_SPEED * ratio));
        else: 
            self._driveVelocity.velocity = desiredState.speed * ratio / swerve_module_constants.WHEEL_CIR;
            self._driveVelocity.feed_forward = self._feedforward.calculate(desiredState.speed);
            self._driveMotor.set_control(self._driveVelocity);
        

    def getEncoderAngle(self) -> Rotation2d:
        return Rotation2d.fromRotations(self._angleEncoder.get_absolute_position().value);


    def getPosition(self) -> SwerveModulePosition:
        return SwerveModulePosition(
            self._driveMotor.get_position().value_as_double * swerve_module_constants.WHEEL_CIR, 
            Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double))