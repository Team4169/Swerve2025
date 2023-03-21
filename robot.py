#!/usr/bin/env python3
import typing
import wpilib
from wpimath.geometry import Rotation2d 
import commands2
import ctre
import math
import constants
from robotcontainer import RobotContainer
from deadzone import addDeadzone
import ntcore
import robotpy_apriltag


class MyRobot(commands2.TimedCommandRobot):
    """
    Our default robot class, pass it to wpilib.run

    Command v2 robots are encouraged to inherit from TimedCommandRobot, which
    has an implementation of robotPeriodic which runs the scheduler for you
    """

    autonomousCommand: typing.Optional[commands2.Command] = None

    def robotInit(self) -> None:
        
        """
        This function is run when the robot is first started up and should be used for any
        initialization code.
        """
        
        # Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        # autonomous chooser on the dashboard.
        # wpilib.CameraServer().launch("vision.py:main")
        self.container = RobotContainer()

        self.driverController = self.container.driverController
        self.operatorController = self.container.operatorController

        self.leftTalon = self.container.leftTalon
        self.leftTalon2 = self.container.leftTalon2
        self.rightTalon = self.container.rightTalon
        self.rightTalon2 = self.container.rightTalon2

        self.arm = self.container.arm
        
        self.drive = self.container.drive

        #~ LED commands and variables
        self.LEDserver = wpilib.I2C(wpilib.I2C.Port.kMXP, 100)
        self.previousLEDCommand = 0
        self.team = "red" #^ change this to blue if we are on the blue team

    def disabledInit(self) -> None:
        """This function is called once each time the robot enters Disabled mode."""


    def disabledPeriodic(self) -> None:
        """This function is called periodically when disabled"""

    def autonomousInit(self) -> None:
        self.drive.gyro.reset()
        self.arm.initializeDegreesOnStart()
        """This autonomous runs the autonomous command selected by your RobotContainer class."""
        self.autonomousCommand = self.container.getAutonomousCommand()
        
        # self.output("ato com", self.autonomousCommand)
        #
        if self.autonomousCommand:
            self.autonomousCommand.schedule()

    def autonomousPeriodic(self) -> None:
        """This function is called periodically during autonomous"""


    def teleopInit(self) -> None:
        wpilib.CameraServer.launch()
        # self.arm.initializeDegreesOnStart()
        self.sendLEDCommand(1, self.team)
        self.drive.resetEncoders()
        self.drive.gyro.reset()
        
        # self.arm.resetGrabbingArmEncoder()
        # self.arm.resetExtendingArmEncoder()
        


        # This makes sure that the autonomous stops running when
        # teleop starts running. If you want the autonomous to
        # continue until interrupted by another command, remove
        # this line or comment it out.
        if self.autonomousCommand:
            self.autonomousCommand.cancel()

        # print("Starting teleop...")
        self.humancontrol = True
        self.speed = 0
        self.intake = 0
        self.outtake = 0
        self.climbMode = False
        self.direction = 0

        self.areaPost = self.container.sd.getDoubleTopic("april_area").publish()
        self.yawPost = self.container.sd.getDoubleTopic("april_yaw").publish()
        self.idPost = self.container.sd.getDoubleTopic("april_id").publish()

    def teleopPeriodic(self):
        self.drive.gyroOut.set(self.drive.gyro.getYaw())
        self.drive.gyroPitchOut.set(self.drive.gyro.getPitch())
        self.arm.updateDegreesAndPercent()
        # self.drive.encoderTicks.set(self.drive.)
        self.drive.encoderRightOut.set(self.rightTalon.getSelectedSensorPosition())
        self.drive.encoderLeftOut.set(self.leftTalon.getSelectedSensorPosition())
        
        #~ smartDashboard limit switch setting
        self.arm.shouldMoveVal.set(self.arm.shouldMove)

        self.arm.grabbingArmLimitSwitchClosedVal.set(self.arm.getGrabbingArmLimitSwitchClosedPressed())
        self.arm.grabbingArmLimitSwitchOpenVal.set(self.arm.getGrabbingArmLimitSwitchOpenPressed())
        self.arm.extendingArmLimitSwitchMinVal.set(self.arm.getExtendingArmLimitSwitchMinPressed())
        self.arm.extendingArmLimitSwitchMaxVal.set(self.arm.getExtendingArmLimitSwitchMaxPressed())
        self.arm.rotatingArmLimitSwitchMaxVal.set(self.arm.getRotatingArmLimitSwitchMaxPressed())
        self.arm.rotatingArmLimitSwitchMinVal.set(self.arm.getRotatingArmLimitSwitchMinPressed())

        self.arm.rotatingArmEncoderDegreesVal.set(self.arm.rotatingArmEncoderDegrees)
        self.arm.extindingArmPercentVal.set(self.arm.extendingArmEncoderPercent)
        

        self.arm.grabbingDegrees.set(self.arm.grabbingArmEncoderDegrees )
        self.arm.extendingArmRevolutions.set(self.arm.extendingArmEncoder.getPosition())
        self.arm.rotatingArmRevolutions.set(self.arm.rotatingArmEncoder.getPosition())
    
    #todo: decide which controller this is on
    #^ untested auto cone pickup code
        self.distance = constants.testDistance
        self.hypot = ((self.distance + constants.cameraDistanceFromArm)**2 + (constants.pivotDistanceFromGround - constants.armPickupHeight)**2)**.5
        if self.driverController.getLeftTriggerAxis() > .1:
            if self.hypot > constants.maxArmLength - 5: #5 is a buffer
                self.drive.driveMecanum(.25, 0, 0)
                # self.setRotatingArmAngle(self.getTargetAngle(self.distance), .75)
            elif self.hypot < constants.minArmLength + 5:
                self.drive.driveMecanum(-.25, 0, 0)
            else:
                self.drive.driveMecanum(0, 0, 0)
                targetAngle = -math.atan((constants.pivotDistanceFromGround - constants.armPickupHeight)/(self.distance + constants.cameraDistanceFromArm)) * 180/math.pi
                if targetAngle < constants.lowerArmAngleLimit:
                    self.arm.setRotatingArmSpeed(0)
                    self.driveMeacanum(-.25, 0, 0)
                else:
                    self.arm.setRotatingArmAngle(targetAngle, .25)
                    # self.arm.setExtendingArmPercentWithAuto((self.hypot / constants.maxArmLength) * 100)
                # self.setRotatingArmAngle(self.getTargetAngle(self.distance), .75)
            #     self.
    #
    #^ start of driving code
        self.leftX = addDeadzone(self.driverController.getLeftX() * (constants.moveRestriction)) #/2 is to slow down the robot
        self.leftY = addDeadzone(self.driverController.getLeftY() * (constants.moveRestriction))
        self.rightX = addDeadzone(self.driverController.getRightX() * (constants.moveRestriction))
        
        self.moving = self.leftX != 0 or self.leftY != 0 or self.rightX != 0

        # if self.driverController.getBButton():
        #     self.arm.setGrabbingArmSpeed(0.1)
        # elif self.driverController.getXButton():
        #     self.arm.setGrabbingArmSpeed(-0.1)
        # elif self.driverController.getYButton():
        #     self.arm.setGrabbingArmAngle(90, 0.09)
        # elif self.driverController.getAButton():
        #     self.arm.setGrabbingArmAngle(45, 0.09)
        # else:
        #     self.arm.setGrabbingArmSpeed(0)
            
        self.flipped = 1
    #^ balancing with the A button
        self.container.drive.balanceSensitivitySub.get()
        self.gyroRad = self.container.drive.gyro.getYaw() * (math.pi/180)
        if self.driverController.getAButton():
            self.pitchAngle = self.container.drive.gyro.getPitch()
            self.speed = constants.maxBalanceSpeed*2/(1 + math.e**(-constants.balanceSensitivity*(self.pitchAngle/constants.maxBalanceAngle)))-constants.maxBalanceSpeed #min(max(-abs(self.pitchAngle) + , 0), 1)
            # maybe make it drive cartesian so that the robot can balance while sideways
            if .05 > self.speed and self.speed > -.05:
                self.sendLEDCommand(7)
            
            #self.drive.driveMecanum(self.speed, 0 , 0, Rotation2d(self.gyroRad)
            self.leftTalon.set(self.speed) 
            self.rightTalon.set(self.speed)
            self.leftTalon2.set(self.speed)
            self.rightTalon2.set(self.speed)
        elif self.driverController.getYButton() or self.driverController.getBButton():
            result = self.container.camera.getLatestResult()

            targetYaw = -0.487
            targetArea =    0.0497

            targets = result.getTargets()
            targetinfo = []
            for i in targets:
                self.idPost.set(i.getFiducialId())
                self.areaPost.set(i.getPitch())
                self.yawPost.set(i.getYaw())
                targetinfo.append({"id": i.getFiducialId(), "area" : i.getArea() * math.pi / 180, "yaw" : i.getYaw() * math.pi / 180})

            xout = 0
            yout = 0

            newtargetinfo = []
            if len(targetinfo) > 0:
                for i in targetinfo:
                    if i["id"] in [1, 2, 3, 6, 7, 8]:
                        newtargetinfo.append(i.copy())
                        
            if len(newtargetinfo) > 0:
                '''
                if self.driverController.getYButton():
                    self.arm.setRotatingArmAngle(27.8, .5)
                    self.arm.setExtendingArmPercentWithAuto(100, .5)
                if self.driverController.getBButton():
                    self.arm.setRotatingArmAngle(28, .5)
                    self.arm.setExtendingArmPercentWithAuto(18, .5)
                '''

                if newtargetinfo[0]["yaw"] > targetYaw:
                    xout = -0.2
                else:
                    xout = 0.2

                if newtargetinfo[0]["area"] > targetArea:
                    yout = -0.2
                else:
                    yout = 0.2
                '''
                if abs(newtargetinfo[0]["yaw"] - targetYaw) < 0.02:
                    xout = 0
                if abs(newtargetinfo[0]["area"] - targetArea) < 0.02:
                    yout = 0
                '''
                    
            self.drive.driveMecanum(xout, yout, 0)
        else:
            self.drive.driveMecanum( -self.leftY * self.flipped, self.leftX * self.flipped, self.rightX, Rotation2d(self.gyroRad)) #self.gyroRad

        # * arm control  
        #Todo: make sure the "foreward" is positive
    
    #^: arm functions
        # self.arm.setRotatingArmSpeedWithAuto(self.operatorController.getLeftY())
        if not self.driverController.getBButton():
            self.arm.setRotatingArmSpeedWithAuto(addDeadzone(self.operatorController.getLeftY() / (2)))
            self.arm.setExtendingArmSpeedWithAuto(addDeadzone(self.operatorController.getRightY() / (4/3)))
            self.arm.setGrabbingArmSpeed(addDeadzone((self.operatorController.getRightTriggerAxis() - self.operatorController.getLeftTriggerAxis()) *(3/4)))

        if self.operatorController.getAButton():
            self.arm.zeroExtendingArm()
        if self.operatorController.getBButton():
            self.arm.zeroRotatingArm()
        if self.operatorController.getYButton():
            self.arm.zeroGrabbingArm()
        if self.operatorController.getLeftBumper():
            self.arm.grabCone()
        if self.operatorController.getRightBumper():
            self.arm.grabCube()

    def testInit(self) -> None:
        # Cancels all running commands at the start of test mode
        commands2.CommandScheduler.getInstance().cancelAll()
    
    def sendLEDCommand(self, command, team = None):
            # send the specified command to the LEDserver
            team_command = command
            if team == "blue":
                team_command = command + 3
            if self.previousLEDCommand != team_command:
                self.previousLEDCommand = team_command
                if self.LEDserver.writeBulk(memoryview(bytes([team_command]))):
                    print("Got an error sending command ", team_command)
                    return True
                else:
                    print("Success sending command ", team_command)
                    return False


if __name__ == "__main__":
    wpilib.run(MyRobot)
# #!/usr/bin/env python3
