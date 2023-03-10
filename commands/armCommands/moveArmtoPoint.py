from subsystems.armsubsystem import ArmSubsystem
import robotcontainer
import constants
import commands2


class MoveArmToPoint(commands2.CommandBase):
    def __init__(self, distance:float, speed:float, arm: ArmSubsystem) -> None:
        super().__init__()
        self.distance = distance
        self.speed = speed
        self.arm = arm

    def initialize(self):
        pass

    def execute(self) -> None:
        angle = self.arm.getTargetAngle(distance)
        self.arm.setRotatingArmAngle(angle,speed)
    def end(self, interrupted: bool) -> None:
        self.climb.setLiftArm(0)

    def isFinished(self) -> bool:
        if self.climb.getLiftArmLimitSwitchPressed():
            return True
        if self.above:
            if self.climb.getLiftArmEncoderDistance() > self.tickLocation:
                return True
        else:
            if self.climb.getLiftArmEncoderDistance() < self.tickLocation:
                return True
        return False
