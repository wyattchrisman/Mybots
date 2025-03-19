import pyrosim.pyrosim as pyrosim
import numpy
import random
import os
import time


class SOLUTION:
    def __init__(self, nextAvailableID):
        self.myID = nextAvailableID

        self.weights = numpy.random.rand(3, 2) * 2 - 1
        # print(self.weights)

    def Set_ID(self, nextAvailableID):
        self.myID = nextAvailableID

    def Evaluate(self, directOrGUI):
        pass

    def Start_Simulation(self, directOrGUI):
        self.Create_World()
        self.Generate_Body()
        self.Generate_Brain()

        os.system(f"python3 simulate.py {directOrGUI} {int(self.myID)} &")

    def Wait_For_Simulation_To_End(self):
        fitnessFile = f"fitness{str(self.myID)}.txt"
        while not os.path.exists(fitnessFile):
            time.sleep(0.01)

        fitnessFile = open(fitnessFile, "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()

        os.system(f"rm fitness{str(self.myID)}.txt")


    def Mutate(self):
        randomRow = random.randint(0,2)
        randomCol = random.randint(0,1)

        self.weights[randomRow,randomCol] = random.random() * 2 - 1

    def Create_World(self):
        pyrosim.Start_SDF("world.sdf")

        pyrosim.Send_Cube(name="Box", pos=[-3,3,0.5], size=[1, 1, 1])

        pyrosim.End()

    def Generate_Body(self):
        pyrosim.Start_URDF("body.urdf")

        pyrosim.Send_Cube(name="Torso", pos=[1.5,0,1.5], size=[1, 1, 1])

        pyrosim.Send_Joint(name = "Torso_BackLeg" , parent= "Torso" , child = "BackLeg" , type = "revolute", position = [1,0,1])
        pyrosim.Send_Cube(name="BackLeg", pos=[-0.5,0,-0.5], size=[1, 1, 1])
        
        pyrosim.Send_Joint(name = "Torso_FrontLeg" , parent= "Torso" , child = "FrontLeg" , type = "revolute", position = [2,0,1])
        pyrosim.Send_Cube(name="FrontLeg", pos=[0.5,0,-0.5], size=[1, 1, 1])

        pyrosim.End()

    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "BackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "FrontLeg")

        pyrosim.Send_Motor_Neuron(name = 3 , jointName = "Torso_BackLeg")
        pyrosim.Send_Motor_Neuron(name = 4 , jointName = "Torso_FrontLeg")

        sensor_neuron_names = list(range(0,3))
        motor_neuron_names = list(range(0,2))

        for currentRow in sensor_neuron_names:
            for currentColumn in motor_neuron_names:
                # random_weight =  (random.random() * 2) - 1
                # print(random_weight)
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn + 3, weight = self.weights[currentRow][currentColumn])
            

        pyrosim.End()

    
   