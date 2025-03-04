import commands2
import wpilib
import wpilib.drive
import constants
from constants import RobotConstants
import ntcore
import rev
import math
import phoenix6

class algaeSubsystem(commands2.SubsystemBase):
    def __init__(self) -> None:
        super().__init__()
        # commands2.SubsystemBase.__init__(self)
        # ~ smartdashboard
        self.sd = wpilib.SmartDashboard

        #* Intake motors
        self.algaeIntakeMotor1 = rev.SparkMax(RobotConstants.algaeIntakeMotor1ID, rev.SparkLowLevel.MotorType.kBrushless)
        self.algaeIntakeMotor2 = rev.SparkMax(RobotConstants.algaeIntakeMotor2ID, rev.SparkLowLevel.MotorType.kBrushless)

        self.algaeLiftMotor = rev.SparkMax(RobotConstants.liftMotorID, rev.SparkLowLevel.MotorType.kBrushless)

    def runAlgae(self, speed: float):
        self.algaeIntakeMotor1.set(speed)
        self.algaeIntakeMotor2.set(speed)
    
    def stopAlgae(self):
        self.algaeIntakeMotor1.set(0)
        self.algaeIntakeMotor2.set(0)

    def runLiftAlgae(self, speed: float):
        self.algaeLiftMotor.set(speed)

    def stopLiftAlgae(self):
        self.algaeLiftMotor.set(0)