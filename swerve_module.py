import math

from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.controller import SimpleMotorFeedforwardMeters
from wpimath.geometry import Rotation2d
import wpimath.units as units

from phoenix6 import hardware, controls

import configs

class swerve_module_constants:
    WHEEL_CIR = units.inchesToMeters(3.94) * math.pi
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

class swerve_module:
    def __init__(self, mod_id, mod_consts):
        self._id = mod_id
        self._offset = mod_consts._offset
        
        self._angleEncoder = hardware.CANcoder(mod_consts._encoderID)
        self._angleEncoder.configurator.apply(configs.swerve.cancoder)

        self._angleMotor = hardware.TalonFX(mod_consts._angleID, "rio")
        self._angleMotor.configurator.apply(configs.swerve.angle)

        self.resetToAbsolute();

        self._driveMotor = hardware.TalonFX(mod_consts._driveID, "rio");
        self._driveMotor.configurator.apply(configs.swerve.drive)

        self._lastAngle = self.getState().angle;
    
        self._feedforward = SimpleMotorFeedforwardMeters(0.667 / 12,2.44 / 12,0.27 / 12)

        self._driveOutput = controls.DutyCycleOut(0)
        self._driveVelocity = controls.VelocityVoltage(0)
        self._anglePosition = controls.PositionVoltage(0)

    def resetToAbsolute(self):
        absolutePosition = self.getEncoderAngle() - self._offset;
        self._angleMotor.set_position(absolutePosition.degrees()/360);

    def getState(self):
        return SwerveModuleState(
            self._driveMotor.get_velocity().value_as_double * swerve_module_constants.WHEEL_CIR, 
            Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double)
        )
    
    def setDesiredState(self, desiredState, openLoop):
        desiredState = SwerveModuleState.optimize(desiredState, self.getState().angle)
        angle = self._lastAngle if math.abs(desiredState.speedMetersPerSecond) <= (swerve_module_constants.MAX_SPEED * 0.01) else desiredState.angle;
        self._angleMotor.set_control(self._anglePosition.position(angle.degrees()/360));
        self.setSpeed(desiredState, openLoop);
        self._lastAngle = angle;

    def setSpeed(self, desiredState, openLoop):
        ratio = (desiredState.angle - self.getState().angle).cos(); 
        if (openLoop):
            self._driveMotor.set_control(self._driveOutput.output(desiredState.speedMetersPerSecond / swerve_module_constants.MAX_SPEED * ratio));
        else: 
            self._driveVelocity.velocity = desiredState.speedMetersPerSecond * ratio / swerve_module_constants.WHEEL_CIR;
            self._driveVelocity.feed_forward = self._feedforward.calculate(desiredState.speedMetersPerSecond);
            self._driveMotor.set_control(self._driveVelocity);
        

    def getEncoderAngle(self) -> Rotation2d:
        return Rotation2d.fromRotations(self._angleEncoder.get_absolute_position().value_as_double);


    def getPosition(self):
        return SwerveModulePosition(
            self._driveMotor.get_position().value_as_double * swerve_module_constants.WHEEL_CIR, 
            Rotation2d.fromRotations(self._angleMotor.get_position().value_as_double)
        )