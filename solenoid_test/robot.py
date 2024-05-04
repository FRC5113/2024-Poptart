import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.solenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 0)
        self.xbox = wpilib.XboxController(0)

    def teleopPeriodic(self):
        if self.xbox.getAButton():
            self.solenoid.set(DoubleSolenoid.Value.kForward)
        elif self.xbox.getBButton():
            self.solenoid.set(DoubleSolenoid.Value.kReverse)
        else:
            self.solenoid.set(DoubleSolenoid.Value.kOff)


if __name__ == "__main__":
    wpilib.run(MyRobot)
