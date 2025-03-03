import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:

    def __init__(self, jointName) -> None:
        self.jointName = jointName
        self.Prepare_To_Act()

    def Set_Value(self, robot, desiredAngle):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=robot.robotId,
            jointName=self.jointName,  
            controlMode=p.POSITION_CONTROL,
            targetPosition=desiredAngle,  
            maxForce=30
        )
