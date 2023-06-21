#
# The constants module is a convenience place for teams to hold robot-wide
# numerical or boolean RobotConstants. Don't use this for any other purpose!
#

import math
import wpiutil
import UtilCommands
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry._geometry import Translation2d
from wpimath.trajectory import TrapezoidProfileRadians, TrapezoidProfile
#~ Controller Options
class OIConstants:
    kDriverControllerPort = 0
    kArmControllerPort = 1
    deadzone = .1

#~ robot specifications
class RobotConstants:
        
    kTrackWidth = UtilCommands.inchesToMeters(30) #todo: find the actual track width 
        # ? Distance between the right and left wheels
    kWheelBase = UtilCommands.inchesToMeters(30) #todo: find the actual wheel base
        # ? Distance between the front and back wheels
    kDriveKinematics = SwerveDrive4Kinematics(
        Translation2d(kTrackWidth / 2, kWheelBase / 2), #frontright
        Translation2d(kTrackWidth / 2, -kWheelBase / 2), #backright
        Translation2d(-kTrackWidth / 2, kWheelBase / 2), #frontleft
        Translation2d(-kTrackWidth / 2, -kWheelBase / 2), # backleft
        # ? location of each swerve module relative to the center of the robot
    )

    frontLeftDrivingMotorID = 1
    frontLeftTurningMotorID = 11
    frontLeftDrivingMotorReversed = False
    frontLeftTurningMotorReversed = False
    frontLeftAbsoluteEncoderId = 1
    frontLeftAbsoluteEncoderOffset = 0
    frontLeftAbsoluteEncoderReversed = False

    frontRightDrivingMotorID = 2
    frontRightTurningMotorID = 22
    frontRightDrivingMotorReversed = False
    frontRightTurningMotorReversed = False
    frontRightAbsoluteEncoderId = 2
    frontRightAbsoluteEncoderOffset = 0
    frontRightAbsoluteEncoderReversed = False

    backLeftDrivingMotorID = 3
    backLeftTurningMotorID = 33
    backLeftDrivingMotorReversed = False
    backLeftTurningMotorReversed = False
    backLeftAbsoluteEncoderId = 3
    backLeftAbsoluteEncoderOffset = 0
    backLeftAbsoluteEncoderReversed = False

    backRightDrivingMotorID = 4
    backRightTurningMotorID = 44
    backRightDrivingMotorReversed = False
    backRightTurningMotorReversed = False
    backRightAbsoluteEncoderId = 4
    backRightAbsoluteEncoderOffset = 0
    backRightAbsoluteEncoderReversed = False

    kPhysicalMaxAngularSpeedRadiansPerSecond  = 2 * 2 * math.pi
    kphysicalMaxSpeedMetersPerSecond = 12 #^ not sure how to get this, maybe look online and multiply by gear ratio?

    kTeleopDriveMaxAccelerationUnitsPerSec = 1 # ? 0 to 1, represents move restriciton
    kTeleopDriveMaxAngularAccelerationUnitsPerSec = 1

    kTeleopDriveMaxSpeedMetersPerSecond = 12 
    kTeleopDriveMaxAngularSpeedRadiansPerSecond = 12

# ~ Swerve Constants 


#todo Change above
class ModuleConstants:
    kWheeleDiameterMeters = UtilCommands.inchesToMeters(4) #^ wheele listed as "Wheel, Billet, 4"OD x 1.5"W (MK4/4i)"" I think that's what the 4 OD means

    kDriveMotorGearRatio = 1/5.43 #^placeholder
    kTurningMotorGearRatio = 1/5.43 #^placeholder

    kDrivingEncoderRot2Meter = kDriveMotorGearRatio * math.pi * kWheeleDiameterMeters
    KDrivingEncoderRPM2MeterPerSec = kDrivingEncoderRot2Meter / 60

    kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * math.pi
    kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60 

    kPTurning = .5 #? turning PID controller per wheele


# ~ Auto Constants

class AutoConstants:
        kMaxSpeedMetersPerSecond = .75
        kMaxAccelerationMetersPerSecondSquared = .75
        kMaxAngularAccelerationRadiansPerSecondSquared = math.pi / 4

        kPXController = 1.0
        kPYController = 1.0

        kMaxAngularSpeedRadiansPerSecond = RobotConstants.kphysicalMaxSpeedMetersPerSecond / 10

        kPThetaController = 1.0
        kThetaControllerConstraints = TrapezoidProfileRadians.Constraints(
                kMaxAngularSpeedRadiansPerSecond,
                kMaxAngularAccelerationRadiansPerSecondSquared,
        )

