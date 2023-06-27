import wpilib.simulation
from pyfrc.physics.core import PhysicsInterface
from pyfrc.physics import drivetrains
import constants
import typing

if typing.TYPE_CHECKING:
    from robot import MyRobot


class PhysicsEngine:
    """
    Simulates a 4-wheel mecanum robot using Tank Drive joystick control
    """

    def __init__(self, physics_controller: PhysicsInterface, robot: "MyRobot"):
        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        """

        self.physics_controller = physics_controller
        # Motors
        self.lf_motor = wpilib.simulation.PWMSim(robot.swerve.frontLeft.drivingMotor.getDeviceId())
        self.lf_turning_motor = wpilib.simulation.PWMSim(robot.swerve.frontLeft.turningMotor.getDeviceId())
        
        self.lr_motor = wpilib.simulation.PWMSim(robot.swerve.backLeft.drivingMotor.getDeviceId())
        self.lr_turning_motor = wpilib.simulation.PWMSim(robot.swerve.backLeft.turningMotor.getDeviceId())
        
        self.rf_motor = wpilib.simulation.PWMSim(robot.swerve.frontRight.drivingMotor.getDeviceId())
        self.rf_turning_motor = wpilib.simulation.PWMSim(robot.swerve.frontRight.turningMotor.getDeviceId())
        
        self.rr_motor = wpilib.simulation.PWMSim(robot.swerve.backRight.drivingMotor.getDeviceId())
        self.rr_turning_motor = wpilib.simulation.PWMSim(robot.swerve.backRight.turningMotor.getDeviceId())
        # Gyro
        self.gyro = wpilib.simulation.AnalogGyroSim(wpilib.SerialPort.Port.kUSB1)

    def update_sim(self, now: float, tm_diff: float) -> None:
        """
        Called when the simulation parameters for the program need to be
        updated.
        :param now: The current time as a float
        :param tm_diff: The amount of time that has passed since the last
                        time that this function was called
        """

        # Simulate the drivetrain
        lf_motor = self.lf_motor.getSpeed()
        lf_angle = self.lf_turning_motor.getSpeed()
        lr_motor = self.lr_motor.getSpeed()
        lr_angle = self.lr_turning_motor.getSpeed()
        rf_motor = self.rf_motor.getSpeed()
        rf_angle = self.rf_turning_motor.getSpeed()
        rr_motor = self.rr_motor.getSpeed()
        rr_angle = self.rr_turning_motor.getSpeed()

        speeds = drivetrains.four_motor_swerve_drivetrain(lr_motor = lr_motor, 
                                                          rf_motor = rf_motor, 
                                                          rr_motor = rr_motor,
                                                          lf_motor = lf_motor,
                                                          lr_angle = lr_angle,
                                                          rf_angle = rf_angle,
                                                          rr_angle = rr_angle,
                                                          lf_angle = lf_angle
                                                          )
        pose = self.physics_controller.drive(speeds, tm_diff)

        # Update the gyro simulation
        # -> FRC gyros are positive clockwise, but the returned pose is positive
        #    counter-clockwise
        self.gyro.setAngle(-pose.rotation().degrees())