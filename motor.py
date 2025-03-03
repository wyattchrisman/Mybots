import numpy
import constants as c
import pyrosim.pyrosim as pyrosim
import pybullet as p

class MOTOR:

    def __init__(self, jointName) -> None:
        self.jointName = jointName
        self.Prepare_To_Act()
  
    def Prepare_To_Act(self):
        self.amplitude = c.amplitude_back  
        self.frequency = c.frequency_back  
        self.offset = c.phaseOffset_back   

        # Generate motor values
        angles = numpy.linspace(0, 2 * numpy.pi, num=c.size)

        if self.jointName == b'Torso_BackLeg':
            self.motorValues = [self.amplitude * numpy.sin(self.frequency/2 * i + self.offset) for i in angles]
        else:
            self.motorValues = [self.amplitude * numpy.sin(self.frequency * i + self.offset) for i in angles]


    def Set_Value(self, robot, step):
        pyrosim.Set_Motor_For_Joint(
            bodyIndex=robot.robotId,
            jointName=self.jointName,  
            controlMode=p.POSITION_CONTROL,
            targetPosition=self.motorValues[step],  
            maxForce=30
        )
        
    def Save_Values(self):
        filename = f"data/{self.jointName}_motor.npy"
        numpy.save(filename, self.motorValues)    
