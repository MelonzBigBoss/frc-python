from wpimath.geometry import Rotation2d, Translation2d

import commands2

from swerve import swerve

from swerve_module import swerve_module_constants

class TeleopSwerve(commands2.Command):
    strafe = lambda: 0;
    translation = lambda: 0;
    rotation: lambda: 0;

    deadband = 1;
    
    def __init__(self, multiplier=1, openLoop=True, fieldRelative=True):
        self.multiplier = multiplier
        self.openLoop = openLoop
        self.fieldRelative = fieldRelative

        self.addRequirements(swerve.instance())

    def config(x, y, r, db):
        TeleopSwerve.strafe = x
        TeleopSwerve.translation = y
        TeleopSwerve.rotation = r

        TeleopSwerve.deadband = db
    

    def execute(self):
        yAxis = TeleopSwerve.translation()
        xAxis = TeleopSwerve.strafe();
        rAxis = TeleopSwerve.rotation();

        vector = Translation2d(xAxis, yAxis)

        theta : Rotation2d = vector.angle();
        magnitude = min(vector.norm(),1) ** 2;
        if (magnitude <= TeleopSwerve.deadband): magnitude = 0;

        if (abs(rAxis) <= self.deadband): rAxis = 0    

        rot = rAxis * swerve_module_constants.MAX_ANG_SPEED;
        #output = Translation2d(magnitude, theta) * swerve_module_constants.MAX_SPEED * self.multiplier;
        swerve.instance().TeleDrive(vector, rot);

    def isFinished(self) -> bool:
        return True