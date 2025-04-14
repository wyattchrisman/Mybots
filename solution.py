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
        sleep_count = 0
        while not os.path.exists(fitnessFile):
            # Catch race condition - if wait 7.5 seconds
            '''
            if sleep_count > 300:
                self.myID += 1
                fitnessFile = f"fitness{str(self.myID)}.txt"
            #print(f"Looking for {fitnessFile}")
            sleep_count += 1
            '''
            time.sleep(0.01)
            

        fitnessFile = open(fitnessFile, "r")
        self.fitness = float(fitnessFile.read())
        fitnessFile.close()

        os.system(f"rm fitness{str(self.myID)}.txt")
        return sleep_count


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

        if c.robot_type == "PHC":
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
        elif c.robot_type == "PRONK":
            pyrosim.Send_Cube(name="Torso", pos=[0,0,1], size=[1, 3, 1])

            pyrosim.Send_Joint(name = "Torso_LeftFrontLeg" , parent= "Torso" , child = "LeftFrontLeg" , type = "revolute", position = [-0.5,1,1], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="LeftFrontLeg", pos=[-0.5,0,0], size=[1, 0.2, 0.2])
            pyrosim.Send_Joint(name = "LeftFrontLeg_LeftFrontLowerLeg" , parent= "LeftFrontLeg" , child = "LeftFrontLowerLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="LeftFrontLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])
            
            pyrosim.Send_Joint(name = "Torso_RightFrontLeg" , parent= "Torso" , child = "RightFrontLeg" , type = "revolute", position = [0.5,1,1], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="RightFrontLeg", pos=[0.5,0,0], size=[1, 0.2, 0.2])
            pyrosim.Send_Joint(name = "RightFrontLeg_RightFrontLowerLeg" , parent= "RightFrontLeg" , child = "RightFrontLowerLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="RightFrontLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])

            pyrosim.Send_Joint(name = "Torso_LeftBackLeg" , parent= "Torso" , child = "LeftBackLeg" , type = "revolute", position = [-0.5,-1,1], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="LeftBackLeg", pos=[-0.5,0,0], size=[1, 0.2, 0.2])
            pyrosim.Send_Joint(name = "LeftBackLeg_LeftBackLowerLeg" , parent= "LeftBackLeg" , child = "LeftBackLowerLeg" , type = "revolute", position = [-1,0,0], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="LeftBackLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])
            
            pyrosim.Send_Joint(name = "Torso_RightBackLeg" , parent= "Torso" , child = "RightBackLeg" , type = "revolute", position = [0.5,-1,1], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="RightBackLeg", pos=[0.5,0,0], size=[1, 0.2, 0.2])
            pyrosim.Send_Joint(name = "RightBackLeg_RightBackLowerLeg" , parent= "RightBackLeg" , child = "RightBackLowerLeg" , type = "revolute", position = [1,0,0], jointAxis = "0 1 0")
            pyrosim.Send_Cube(name="RightBackLowerLeg", pos=[0,0,-0.5], size=[0.2, 0.2, 1])
        
        
        pyrosim.End()


    def Generate_Brain(self):
        pyrosim.Start_NeuralNetwork(f"brain{self.myID}.nndf")

        pyrosim.Send_Sensor_Neuron(name = 0, linkName = "Torso")
        pyrosim.Send_Sensor_Neuron(name = 1, linkName = "LeftBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 2, linkName = "RightBackLeg")
        pyrosim.Send_Sensor_Neuron(name = 3, linkName = "LeftFrontLeg")        
        pyrosim.Send_Sensor_Neuron(name = 4, linkName = "RightFrontLeg")

        '''
        pyrosim.Send_Sensor_Neuron(name = 5, linkName = "BackLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 6, linkName = "FrontLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 7, linkName = "RightLowerLeg")
        pyrosim.Send_Sensor_Neuron(name = 8, linkName = "LeftLowerLeg")
        '''

        pyrosim.Send_Motor_Neuron(name = 5, jointName = "Torso_LeftBackLeg")
        pyrosim.Send_Motor_Neuron(name = 6, jointName = "Torso_RightBackLeg")
        pyrosim.Send_Motor_Neuron(name = 7, jointName = "Torso_LeftFrontLeg")
        pyrosim.Send_Motor_Neuron(name = 8, jointName = "Torso_RightFrontLeg")
        pyrosim.Send_Motor_Neuron(name = 9, jointName = "LeftBackLeg_LeftBackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 10, jointName = "RightBackLeg_RightBackLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 11, jointName = "LeftFrontLeg_LeftFrontLowerLeg")
        pyrosim.Send_Motor_Neuron(name = 12, jointName = "RightFrontLeg_RightFrontLowerLeg")

        sensor_neuron_names = list(range(0,c.numSensorNeurons))
        motor_neuron_names = list(range(0,c.numMotorNeurons))

        for currentRow in sensor_neuron_names:
            for currentColumn in motor_neuron_names:
                # random_weight =  (random.random() * 2) - 1
                # print(random_weight)
                pyrosim.Send_Synapse(sourceNeuronName = currentRow, targetNeuronName = currentColumn + c.numSensorNeurons, weight = self.weights[currentRow][currentColumn])
            

        pyrosim.End()

    
   