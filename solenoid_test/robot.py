import wpilib
from wpilib import DoubleSolenoid, PneumaticsModuleType, Compressor


class MyRobot(wpilib.TimedRobot):
    def robotInit(self):
        self.compressor = Compressor(PneumaticsModuleType.REVPH)
        self.solenoid = DoubleSolenoid(PneumaticsModuleType.REVPH, 0, 1)
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
