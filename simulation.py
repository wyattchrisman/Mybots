import pybullet as p
import pybullet_data
import pyrosim.pyrosim as pyrosim
import constants as c
import time
from world import WORLD
from robot import ROBOT

class SIMULATION:

    def __init__(self, directOrGUI, solutionID) -> None:

        self.directOrGUI = directOrGUI

        
        if self.directOrGUI == "DIRECT":
            self.physicsClient = p.connect(p.DIRECT)
        else:
            self.physicsClient = p.connect(p.GUI)


        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-9.8, self.physicsClient)

        self.world = WORLD()
        self.robot = ROBOT(solutionID)

    def Run(self):
        for step in range(c.size):
            self.robot.Sense(step)
            self.robot.Think()
            self.robot.Act(step)
            
            p.stepSimulation()
            
            #print(step)
            if self.directOrGUI == "GUI":
                time.sleep(c.sleep)
                # pass

    def Get_Fitness(self):
        self.robot.Get_Fitness()

    def __del__(self):
        for sensor in self.robot.sensors.values():
            sensor.Save_Values()

        '''
        for motor in self.robot.motors.values():
            motor.Save_Values()
        '''    
        
        p.disconnect()

