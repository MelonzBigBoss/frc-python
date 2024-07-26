#!/usr/bin/env python3

import wpilib

import constants 

from commands2 import CommandScheduler

from swerve import swerve

from TeleopSwerve import TeleopSwerve

from controls import driverstation

class MyRobot(wpilib.TimedRobot):

    def robotInit(self):
        self.powerDistribution = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)

        self.powerDistribution.setSwitchableChannel(True)

        self.autoCommand = None;

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        wpilib.LiveWindow.disableAllTelemetry()

        wpilib.RobotController.setEnabled3V3(False)
        wpilib.RobotController.setEnabled5V(True)
        wpilib.RobotController.setEnabled6V(False)

        TeleopSwerve.config(driverstation.leftJoyX, driverstation.leftJoyY, driverstation.rightJoyX, driverstation.deadband)

        swerve.instance().setDefaultCommand(TeleopSwerve())
        wpilib.SmartDashboard.putNumber("hi", 1)
        wpilib.SmartDashboard.putData(swerve.instance())

    def robotPeriodic(self):
        CommandScheduler.getInstance().run();

    def autonomousInit(self):
        """This function is run once each time the robot enters autonomous mode."""


    def autonomousPeriodic(self):
        """ periodic """

    def teleopInit(self):
        """ init """


    def teleopPeriodic(self):
        """ nuts """

    def testInit(self):

        """This function is called once each time the robot enters test mode."""


    def testPeriodic(self):

        """This function is called periodically during test mode."""

if __name__ == "__main__":
    wpilib.run(MyRobot)