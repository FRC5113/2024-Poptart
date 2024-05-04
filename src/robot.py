#!/usr/bin/env python3
import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType
from magicbot import MagicRobot

from components.drivetrain import Drivetrain, OctoMode
import util
from util import WPI_TalonFX


class MyRobot(MagicRobot):
    #
    # Define components here (high level first, low level last)
    #

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""
        self.drivetrain_front_left_motor = util.WPI_TalonFX(0)
        self.drivetrain_front_right_motor = util.WPI_TalonFX(0)
        self.drivetrain_back_left_motor = util.WPI_TalonFX(0)
        self.drivetrain_back_right_motor = util.WPI_TalonFX(0)

        # UPDATE WITH CORRECT IDs!!!!!!
        self.drivetrain_front_left_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 0, 1
        )
        self.drivetrain_front_right_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 2, 3
        )
        self.drivetrain_back_left_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 4, 5
        )
        self.drivetrain_back_right_solenoid = DoubleSolenoid(
            PneumaticsModuleType.REVPH, 6, 7
        )

        self.xbox = wpilib.XboxController(0)

        self.drive_curve = util.cubic_curve(scalar=1, deadband=0.1, max_mag=1)

    def teleopPeriodic(self):
        """Place code here that does things as a result of operator
        actions"""

        with self.consumeExceptions():
            # may want to consider a more sound way of changing modes
            if self.xbox.getXButton():
                self.drivetrain.set_mode(OctoMode.MECANUM_DRIVE)
            if self.xbox.getYButton():
                self.drivetrain.set_mode(OctoMode.DIFFERENTIAL_DRIVE)

            if self.drivetrain.get_mode() == OctoMode.MECANUM_DRIVE:
                self.drivetrain.cartesian_drive(
                    self.drive_curve(self.xbox.getLeftY()),
                    self.drive_curve(self.xbox.getLeftX()),
                    self.drive_curve(-self.xbox.getRightX()),
                )
            if self.drivetrain.get_mode() == OctoMode.DIFFERENTIAL_DRIVE:
                self.drivetrain.arcade_drive(
                    self.drive_curve(self.xbox.getLeftY()),
                    self.drive_curve(-self.xbox.getRightX()),
                )


if __name__ == "__main__":
    wpilib.run(MyRobot)
