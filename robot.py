import pyrosim.pyrosim as pyrosim
import pybullet as p
from pyrosim.neuralNetwork import NEURAL_NETWORK
from sensor import SENSOR
from motor import MOTOR
import constants as c
import os
import numpy
import csv

class ROBOT:

    def __init__(self, solutionID) -> None:
        self.robotId = p.loadURDF("body.urdf")
        self.solutionID = solutionID

        self.legs_touching = 0
        self.legs_floating = 0
        self.legs_mismatch = 0
        self.legs_status = []
        self.torso_status = []
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

        torso_touching = True if self.sensors['Torso'].return_value(step) == 1 else False
        self.torso_status.append(torso_touching)

        all_touching = True if average_sensor == 1 else False
        all_floating = True if average_sensor == -1 else False

        self.legs_touching += 1 if all_touching else 0
        self.legs_floating += 1 if all_floating else 0
        self.legs_mismatch += 1 if not (all_floating or all_touching) else 0
        
        if all_touching:
            current_status = -1
        elif all_floating:
            current_status = 1
        else:   
            current_status = 0

        self.legs_status.append(current_status)

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
        
        basePositionAndOrientation = p.getBasePositionAndOrientation(self.robotId)
        basePosition = basePositionAndOrientation[0]
        xPosition, yPosition, zPosition = basePosition[:3]

        distance_traveled = numpy.sqrt(xPosition**2 + yPosition**2)
        
        # MAXIMIZE FITNESS
        # maximize air time
        # get oscillation
        # want distance
        longest_jump = 0
        current_jump = 0
        for value in self.legs_status:
            if value == 1:
                current_jump += 1
                longest_jump = max(longest_jump, current_jump)
            else:
                current_jump = 0

        # fitness = abs(self.legs_floating - self.legs_touching * 3) + self.legs_mismatch
        # fitness = distance_traveled * 80 - self.legs_touching - self.legs_mismatch
        # fitness = longest_jump

        # Prioritize distance, uniform legs
        # fitness = distance_traveled * 100 + self.legs_floating - self.legs_mismatch * 2 - self.legs_touching

        # Prioritize height, uniform legs
        # fitness = self.highest_z * 400 + self.legs_floating - self.legs_mismatch * 2 - self.legs_touching

        # Prioritize distance, height, uniform legs
        fitness = (self.highest_z * distance_traveled) * 100 + self.legs_floating - self.legs_mismatch * 2 - self.legs_touching
        
        oscillations = 0
        next_step = -1
        for step in self.legs_status:
            if step == next_step:
                next_step *= -1
                oscillations += 1

        # fitness = oscillations * self.highest_z * distance_traveled - self.legs_mismatch

        # Penalize if torso touches ground ever
        torso_touched = True if True in self.torso_status else False
        fitness = -9999 if torso_touched and c.maximize == True else fitness
        fitness = 9999 if torso_touched and c.maximize == False else fitness

        # fitness = -longest_jump * (xPosition + yPosition)

        # print(f"Pre: {fitness}, Post: {post}")

        with open(f"tmp{str(self.solutionID)}.txt", "w") as file:
            file.write(str(fitness))

        with open(f"tmp-oscillation{str(self.solutionID)}.txt", "w") as file:
            file.write(str(oscillations))

        os.system(f"mv tmp{str(self.solutionID)}.txt fitness{str(self.solutionID)}.txt")
        os.system(f"mv tmp-oscillation{str(self.solutionID)}.txt oscillations{str(self.solutionID)}.txt")
