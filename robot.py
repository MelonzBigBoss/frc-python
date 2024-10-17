#!/usr/bin/env python3

import wpilib

import constants

import commands2

from commands2 import CommandScheduler, TimedCommandRobot

from swerve import swerve

from TeleopSwerve import TeleopSwerve

from controls import driverstation

from phoenix6 import hardware
class MyRobot(commands2.TimedCommandRobot):

    def robotInit(self):
        self.powerDistribution = wpilib.PowerDistribution(1, wpilib.PowerDistribution.ModuleType.kRev)

        self.powerDistribution.setSwitchableChannel(True)

        self.autoCommand = None;

        wpilib.DriverStation.silenceJoystickConnectionWarning(True)
        wpilib.LiveWindow.disableAllTelemetry()

        wpilib.RobotController.setEnabled3V3(False)
        wpilib.RobotController.setEnabled5V(True)

        TeleopSwerve.config(driverstation.leftJoyX, driverstation.leftJoyY, driverstation.rightJoyX, driverstation.deadband)

        swerve.instance().setDefaultCommand(TeleopSwerve())

    def robotPeriodic(self):
        """function"""

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