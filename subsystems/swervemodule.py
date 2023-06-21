import commands2
import rev
import wpilib
import math
import wpimath
from wpimath.kinematics import SwerveModuleState, SwerveModulePosition
from wpimath.geometry import Rotation2d
from wpimath.controller import PIDController

from constants import ModuleConstants, RobotConstants
class swervemodule(commands2.SubsystemBase):
    def __init__(self, drivingMotorID: int, turningMotorID:int, drivingMotorReversed: bool, turningMotorReversed: bool, 
                 absoluteEncoderId: int, absouteEncoderOffset: float, absoluteEncoderReversed: bool) -> None:
        super().__init__()

        #* Absolute Encoder
        #~Absolute Encoders help "remember" the location of the module.
        #~This is useful for when the robot is turned off and on again.
        self.absoluteEncoderOffsetRad = absouteEncoderOffset
        self.absoluteEncoderReversed = absoluteEncoderReversed

        self.absoluteEncoder = wpilib.AnalogInput(absoluteEncoderId)
        
        #* Swerve Module Motors init and Encoders
                # self.extendingArm = rev.CANSparkMax(RobotConstants.extendingArmID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.drivingMotor = rev.CANSparkMax(drivingMotorID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)
        self.turningMotor = rev.CANSparkMax(turningMotorID, rev.CANSparkMaxLowLevel.MotorType.kBrushless)

        self.drivingMotor.setInverted(drivingMotorReversed)
        self.turningMotor.setInverted(turningMotorReversed)

        self.drivingEncoder = self.drivingMotor.getEncoder()
        self.turningEncoder = self.turningMotor.getEncoder()
        
        #~ this converts encoder ticks to meters, not entirely sure about how velocity works
        
        self.drivingEncoder.setPositionConversionFactor(ModuleConstants.kDrivingEncoderRot2Meter)
        self.drivingEncoder.setVelocityConversionFactor(ModuleConstants.KDrivingEncoderRPM2MeterPerSec)

        self.turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        self.turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec)

        #* PID Controllers
        self.turningPIDController = PIDController(ModuleConstants.kPTurning, 0, 0)
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)
    
        self.resetEncoders()

        #* smartdashboard init
        self.sd = wpilib.SmartDashboard
    #~ Helper functions

    def getDrivingPosition(self) -> float:
        return self.drivingEncoder.getPosition()
    
    def getTurningPostion(self) -> float:
        return self.turningEncoder.getPosition()
    
    def getDrivingVelocity(self) -> float:
        return self.drivingEncoder.getVelocity()
    
    def getTurningVelocity(self) -> float:
        return self.turningEncoder.getVelocity()
    
    def getAbsoluteEncoderRad(self) -> float:
        angle = self.absoluteEncoder.getVoltage() / wpilib.RobotController.getVoltage5V() #? percent of full rotation
        angle *= 2 * math.pi #? convert to radians
        angle -= self.absoluteEncoderOffsetRad #? get acual location depending on the offset
        return angle * (-1 if self.absoluteEncoderReversed else 1) #? reverse if needed
    
    def getSwerveModulePosition(self) -> SwerveModulePosition:
            return SwerveModulePosition(self.getDrivingPosition(), Rotation2d(self.getAbsoluteEncoderRad()))

    def resetEncoders(self):
        self.drivingEncoder.setPosition(0)
        self.turningEncoder.setPosition(self.getAbsoluteEncoderRad())

    def getState(self) -> SwerveModuleState:
        return SwerveModuleState(self.getDrivingVelocity(), Rotation2d(self.getTurningPostion()))

    def setDesiredState(self, state:SwerveModuleState):
        #^ this prevents the wheels from resetting their position after every input is made
        if (abs(state.speed) < 0.001):
            self.stop()
            return
        self.state = SwerveModuleState.optimize(state, self.getState().angle)
        self.drivingMotor.set(self.state.speed / RobotConstants.kphysicalMaxSpeedMetersPerSecond)
        self.turningMotor.set(self.turningPIDController.calculate(self.getTurningPostion(), self.state.angle.radians()))
        self.sd.putString(f"Swerve[{self.absoluteEncoder.getChannel}] state", str(self.state))

    def stop(self):
        self.drivingMotor.set(0)
        self.turningMotor.set(0)