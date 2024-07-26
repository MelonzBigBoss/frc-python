import wpilib
import commands2

from wpimath.estimator import SwerveDrive4PoseEstimator
from wpimath.geometry import Rotation2d, Pose2d
from wpimath.kinematics import SwerveDrive4Kinematics, ChassisSpeeds
from wpimath.geometry import Translation2d

from phoenix6 import hardware, configs

import constants

from swerve_module import swerve_module, swerve_module_constants

from Singleton import Singleton

@Singleton
class swerve(commands2.Subsystem):

    def __init__(self) -> None:
        super().__init__()

        self.modules = { "front Left":  swerve_module(0, swerve_module_constants(0,6,3,-110.52)),
                            "front Right": swerve_module(1, swerve_module_constants(3,7,4,96.48)), 
                            "back Left":   swerve_module(2, swerve_module_constants(2,1,2,133.56)), 
                            "back Right":  swerve_module(3, swerve_module_constants(1,0,5,128.52)) }
        
        self.kinematics = SwerveDrive4Kinematics(Translation2d(constants.swerve.wheelX / 2.0, constants.swerve.wheelY / 2.0), 
                                                Translation2d(constants.swerve.wheelX / 2.0, -constants.swerve.wheelY / 2.0), 
                                                Translation2d(-constants.swerve.wheelX / 2.0, constants.swerve.wheelY / 2.0), 
                                                Translation2d(-constants.swerve.wheelX / 2.0, -constants.swerve.wheelY / 2.0));

        self._pigeon = hardware.Pigeon2(0); 
        self._pigeon.configurator.apply(configs.Pigeon2Configuration())

        self._yawOffset = Rotation2d(0);

        self._swervePositions = [None, None, None, None];
        
        self.updateModulePositions();

        self._poseEstimator = SwerveDrive4PoseEstimator(self.kinematics, self.getYaw(), tuple(self._swervePositions), Pose2d());
    
        self._field = wpilib.Field2d()
    
    def getYaw(self) -> Rotation2d:
        return Rotation2d.fromDegrees(-1 * self._pigeon.get_yaw().value_as_double); 

    def zeroYaw(self):
        self._yawOffset = self.getYaw();
        self.resetPose(Pose2d(self.getPose().getTranslation(), 0 )); #Rotation2d.fromDegrees(Util.getAlliance() == DriverStation.Alliance.Red ? 180 : 0)

    def resetPose(self, pose):
        self._poseEstimator.resetPosition(self.getYaw(), tuple(self._swervePositions), pose);

    def getPose(self):
        return self._poseEstimator.getEstimatedPosition();
  

    def updateModulePositions(self):
        for index, key, in enumerate(self.modules):
            self._swervePositions[index] = self.modules[key].getPosition();
    
        return tuple(self._swervePositions)
    
    def setModuleStates(self, desiredStates, openLoop = False):
        SwerveDrive4Kinematics.desaturateWheelSpeeds(desiredStates, swerve_module_constants.MAX_SPEED);
    
        for index, key in enumerate(self.modules):
            self.modules[key].setDesiredState(desiredStates[index], openLoop);
        
    def Drive(self, xy, r, openLoop = False, fieldRelative = True):
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            ChassisSpeeds.fromFieldRelativeSpeeds(xy.X(), xy.Y(), r, self.getYaw() + self._yawOffset) #Rotation2d.fromDegrees( (Util.getAlliance() == Alliance.Red ? 180 : 0)
                if fieldRelative else
            ChassisSpeeds(xy.X(), xy.Y(), r))
            
        self.setModuleStates(swerveModuleStates, openLoop);

    def Drive(self, c, openLoop = False):
        self.setModuleStates(self.kinematics.toSwerveModuleStates(c), openLoop)

    def periodic(self):
        self._poseEstimator.update(
            self.getYaw(),
            self.updateModulePositions())
        
        self._field.setRobotPose(self.getPose())

    def initSendable(self, builder):
        builder.addDoubleProperty("yaw", lambda: self.getYaw().degrees(), lambda x: None)

        for index, key in enumerate(self.modules):
            builder.addDoubleProperty(key, lambda: self.modules[key].getEncoderAngle().degrees(), lambda x: None)


        #wpilib.SmartDashboard.putData(self._field)


        

    