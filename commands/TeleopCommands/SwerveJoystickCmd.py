import commands2, constants, wpilib, navx, threading, time, math
from constants import OIConstants, RobotConstants
from wpimath.filter import SlewRateLimiter
from commands2 import CommandBase 

from subsystems.swervesubsystem import SwerveSubsystem
from wpimath.kinematics import ChassisSpeeds

class SwerveJoystickCmd(CommandBase):

    def __init__(self, swerve: SwerveSubsystem, xSpeed:float, 
                    YSpeed:float, ZRotation:float, feildOriented:bool):
        super().__init__()
        self.swerve = swerve
        self.xSpeed = xSpeed
        self.YSpeed = YSpeed
        self.ZRotation = ZRotation
        self.feildOriented = feildOriented
        self.addRequirements(self.swerve)

        # create Slew limiter
        self.xLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationUnitsPerSec)
        self.yLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationUnitsPerSec)
        self.zRotLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAngularAccelerationUnitsPerSec)



    def initialize(self):
        pass

    def execute(self):
        # 1. Get the joystick values and apply deadzone
        self.xSpeed = self.xSpeed if (abs(self.xSpeed) > OIConstants.deadzone) else 0
        self.YSpeed = self.YSpeed if (abs(self.YSpeed) > OIConstants.deadzone) else 0
        self.ZRotation = self.ZRotation if (abs(self.ZRotation) > OIConstants.deadzone) else 0
        
        # 2. Add rateLimiter to smooth the joystick values
        self.xSpeed = self.xLimiter.calculate(self.xSpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.YSpeed = self.yLimiter.calculate(self.YSpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ZRotation = self.zRotLimiter.calculate(self.ZRotation) \
                    * RobotConstants.kTeleopDriveMaxAngularSpeedRadiansPerSecond
        if self.feildOriented:
            chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.YSpeed, 
                                                                 self.ZRotation, self.swerve.getRotation2d())
        else:
            chasisSpeeds = ChassisSpeeds(self.xSpeed, self.YSpeed, self.ZRotation)

        # 3. convert chasis speeds to module states
        moduleStates = RobotConstants.kDriveKinematics.toSwerveModuleStates(chasisSpeeds)

        self.swerve.setModuleStates(moduleStates)
        

    def end(self, interrupted):
        self.swerve.stopModules()

    def isFinished(self):
        return False