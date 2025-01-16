import commands2, constants, wpilib, navx, threading, time, math
from constants import OIConstants, RobotConstants, DrivingConstants
from wpimath.filter import SlewRateLimiter
from commands2 import Command 
from wpilib import XboxController
from commands2.button import CommandXboxController
import wpimath
from subsystems.swervesubsystem import SwerveSubsystem
from wpimath.kinematics import ChassisSpeeds

class SwerveJoystickCmd(Command):

    def __init__(self, swerve: SwerveSubsystem, driverController:XboxController):
        super().__init__()
        self.swerve = swerve
        self.driverController = driverController
        self.addRequirements(self.swerve)
        # create Slew limiter
        # self.xLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationMetersPerSecSquared)
        # self.yLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationMetersPerSecSquared)
        # self.zRotLimiter = SlewRateLimiter(RobotConstants.kTeleopDriveMaxAccelerationMetersPerSecSquared)

    def initialize(self):
        pass
    
    def execute(self):
        #these are multiplied by the drivingSpeedLimiter which limit the speed of the robot so it doesn't go too fast
        self.xSpeed = self.driverController.getLeftX() * DrivingConstants.drivingSpeedLimiter #self.drivingLimiter#* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.driverController.getLeftY() * DrivingConstants.drivingSpeedLimiter#self.drivingLimiter #* RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.driverController.getRightX() * -1 * DrivingConstants.rotationSpeedLimiter#self.drivingLimiter
        
        # 1. Get the joystick values and apply deadzone
        
        # print(self.ySpeed)
        # print(self.zRotation)
        
        self.xSpeed = wpimath.applyDeadband(self.xSpeed, OIConstants.deadzone)
        self.ySpeed = wpimath.applyDeadband(self.ySpeed, OIConstants.deadzone)
        self.zRotation = wpimath.applyDeadband(self.zRotation, OIConstants.deadzone)
        
        # self.swerve.sd.putNumber("xSpeed", self.xSpeed) 
        # self.swerve.sd.putNumber("ySpeed", self.ySpeed) 
        # self.swerve.sd.putNumber("Zspeed", self.zRotation)
        # # 2. Add rateLimiter to smooth the joystick values
        self.xSpeed = self.xLimiter.calculate(self.xSpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.ySpeed = self.yLimiter.calculate(self.ySpeed) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond
        self.zRotation = self.zRotLimiter.calculate(self.zRotation) * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond * RobotConstants.kTeleopDriveMaxSpeedMetersPerSecond / 0.418480
        #! Not sure why this is needed, for some reason without it, the robot rotates slower 

        # if self.feildOriented:
        chasisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(self.xSpeed, self.ySpeed, 
                                                            self.zRotation, self.swerve.getRotation2d())
        
        # else:
        # If robotOrinted is desired (But what's the fun in that?)
        #     chasisSpeeds = ChassisSpeeds(self.xSpeed, self.ySpeed, self.zRotation)

        # 3. convert chasis speeds to module states
        moduleStates = RobotConstants.kDriveKinematics.toSwerveModuleStates(chasisSpeeds)
        # self.swerve.sd.putNumber("ExpectedFL", float(moduleStates[0].angle.degrees()))
        # self.swerve.sd.putNumber("ExpectedFR", float(moduleStates[1].angle.degrees()))
        # self.swerve.sd.putNumber("ExpectedBL", float(moduleStates[2].angle.degrees()))
        # self.swerve.sd.putNumber("ExpectedBR", float(moduleStates[3].angle.degrees()))

        # self.swerve.sd.putNumber("ExpectedSpeedFL", float(moduleStates[0].speed))
        # self.swerve.sd.putNumber("ExpectedSpeedFR", float(moduleStates[1].speed))
        # self.swerve.sd.putNumber("ExpectedSpeedBL", float(moduleStates[2].speed))
        # self.swerve.sd.putNumber("ExpectedSpeedBR", float(moduleStates[3].speed))



        self.swerve.setModuleStates(moduleStates)
        

    def end(self, interrupted):
        self.swerve.stopModules()

    def isFinished(self):
        return False