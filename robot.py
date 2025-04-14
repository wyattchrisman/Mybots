import pyrosim.pyrosim as pyrosim
import pybullet as p
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import constants as c
import os
import numpy

class ROBOT:

    def __init__(self, solutionID) -> None:
        self.robotId = p.loadURDF("body.urdf")
        self.solutionID = solutionID
        self.legs_touching = 0
        self.legs_floating = 0
        self.legs_mismatch = 0

        self.highest_z = 0

        pyrosim.Prepare_To_Simulate(self.robotId)

        self.nn = NEURAL_NETWORK(f"brain{solutionID}.nndf")

        self.Prepare_To_Sense()
        self.Prepare_To_Act()

        os.system(f"rm brain{self.solutionID}.nndf")

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)
            

    def Sense(self, step):
        values = []
        for sensorName, sensor in self.sensors.items():
            sensor.Get_Value(step)
            if 'Lower' in sensorName:
                values.append(sensor.return_value(step))

        average_sensor = numpy.mean(values)

        all_touching = True if average_sensor == 1 else False
        all_not_touching = True if average_sensor == -1 else False

        self.legs_touching += 1 if all_touching else 0
        self.legs_floating += 1 if all_not_touching else 0
        self.legs_mismatch += 1 if not (all_not_touching or all_touching) else 0

        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zPosition = basePosition[2]

        self.highest_z = zPosition if zPosition > self.highest_z else self.highest_z

        #print(f"Step {step}: sensor values {[int(i) for i in values]}")
        #print(f"Step {step}: average = {average_sensor}, all legs matching is {all_touching}")

    def Prepare_To_Act(self):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName) 

    def Act(self, step):
        for neuronName in self.nn.Get_Neuron_Names():
            
            if self.nn.Is_Motor_Neuron(neuronName):
                jointName = self.nn.Get_Motor_Neurons_Joint(neuronName).encode("utf-8")

                desiredAngle = self.nn.Get_Value_Of(neuronName) * c.motorJointRange

                self.motors[jointName].Set_Value(self, desiredAngle)

                # print(neuronName, jointName, desiredAngle)

        '''
        for jointName, motor in self.motors.items():
            motor.Set_Value(self, step)
        '''

    def Think(self):
        self.nn.Update()
        #self.nn.Print()

    def Get_Fitness(self):
        '''
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        zPosition = basePosition[2]
        '''

        fitness = abs(self.legs_floating - self.legs_touching * 3) + self.legs_mismatch

        #fitness = fitness - (self.highest_z * 100)

        # print(f"Pre: {fitness}, Post: {post}")

        with open(f"tmp{str(self.solutionID)}.txt", "w") as file:
            file.write(str(fitness))

        os.system(f"mv tmp{str(self.solutionID)}.txt fitness{str(self.solutionID)}.txt")
