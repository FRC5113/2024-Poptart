#!/usr/bin/env python3
import wpilib
from wpimath.geometry import Translation2d
from wpimath.kinematics import DifferentialDriveKinematics, MecanumDriveKinematics
from magicbot import MagicRobot

from components.drivetrain import Drivetrain, OctoMode, OctoModule
import util


class MyRobot(MagicRobot):
    #
    # Define components here (high level first, low level last)
    #

    drivetrain: Drivetrain

    def createObjects(self):
        """Initialize all wpilib motors & sensors"""

        # UPDATE WITH CORRECT IDs!!!!!!
        self.drivetrain_front_left_module = OctoModule(14, 7, 8)
        self.drivetrain_front_right_module = OctoModule(16, 1, 14, isInverted=True)
        self.drivetrain_back_left_module = OctoModule(15, 6, 9)
        self.drivetrain_back_right_module = OctoModule(17, 0, 15, isInverted=True)
        self.drivetrain_differential_kinematics = DifferentialDriveKinematics(0.56)
        self.drivetrain_mecanum_kinematics = MecanumDriveKinematics(
            Translation2d(0.32, 0.28),
            Translation2d(0.32, -0.28),
            Translation2d(-0.32, 0.28),
            Translation2d(-0.32, -0.28),
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
