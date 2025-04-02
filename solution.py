import pyrosim.pyrosim as pyrosim
import numpy
import random
import os
import time
import constants as c


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID

        self.weights = numpy.random.rand(c.numSensorNeurons, c.numMotorNeurons) * 2 - 1
        # print(self.weights)

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()

        os.system(f"python3 simulate.py {directOrGUI} {int(self.myID)} 2&>1.txt &")

    def Wait_For_Simulation_To_End(self):
        fitnessFile = f"fitness{str(self.myID)}.txt"
        while not os.path.exists(fitnessFile):
            time.sleep(0.01)

        fitnessFile = open(fitnessFile, "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()

        os.system(f"rm fitness{str(self.myID)}.txt")


    def Mutate(self):
        randomRow = random.randint(0,c.numSensorNeurons - 1)
        randomCol = random.randint(0,c.numMotorNeurons - 1)

        self.weights[randomRow,randomCol] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")

        pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5], size=[1, 1, 1])

        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[1, 1, 1])

        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [0,-0.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLeg", pos=[0,-0.5,0], size=[0.2, 1, 0.2])
        
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [0,0.5,1], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLeg", pos=[0,0.5,0], size=[0.2, 1, 0.2])
        
        pyrosim.Send_Joint(name = "Torso_LeftLeg" , parent= "Torso" , child = "LeftLeg" , type = "revolute", position = [-0.5,0,1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLeg", pos=[-0.5,0,0], size=[1, 0.2, 0.2])
        
        pyrosim.Send_Joint(name = "Torso_RightLeg" , parent= "Torso" , child = "RightLeg" , type = "revolute", position = [0.5,0,1], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLeg", pos=[0.5,0,0], size=[1, 0.2, 0.2])

        pyrosim.Send_Joint(name = "FrontLeg_FrontLowerLeg" , parent= "FrontLeg" , child = "FrontLowerLeg" , type = "revolute", position = [0,1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="FrontLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name = "BackLeg_BackLowerLeg" , parent= "BackLeg" , child = "BackLowerLeg" , type = "revolute", position = [0,-1,0], jointAxis = "1 0 0")
        pyrosim.Send_Cube(name="BackLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name = "LeftLeg_LeftLowerLeg" , parent= "LeftLeg" , child = "LeftLowerLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="LeftLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])

        pyrosim.Send_Joint(name = "RightLeg_RightLowerLeg" , parent= "RightLeg" , child = "RightLowerLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
        pyrosim.Send_Cube(name="RightLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])
        
        pyrosim.End()


    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        '''
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftLeg")        
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "RightLeg")
        '''
        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "RightLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftLowerLeg")
        

        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 5 , jointName = "Torso_FrontLeg")
        pyrosim.Send_Motor_Neuron(name = 6 , jointName = "Torso_LeftLeg")
        pyrosim.Send_Motor_Neuron(name = 7 , jointName = "Torso_RightLeg")
        pyrosim.Send_Motor_Neuron(name = 8 , jointName = "BackLeg_BackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 9 , jointName = "FrontLeg_FrontLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = "LeftLeg_LeftLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 11, jointName = "RightLeg_RightLowerLeg")

        sensor_neuron_names = list(range(0,c.numSensorNeurons))
        motor_neuron_names = list(range(0,c.numMotorNeurons))

        for currentRow in sensor_neuron_names:
            for currentColumn in motor_neuron_names:
                # random_weight =  (random.random() * 2) - 1
                # print(random_weight)
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn + c.numSensorNeurons, weight = self.weights[currentRow][currentColumn])
            

        pyrosim.End()

    
   