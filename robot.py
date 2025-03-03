import pyrosim.pyrosim as pyrosim
import pybullet as p
from sensor import SENSOR
from motor import MOTOR
import constants as c
import numpy


class ROBOT:

    def __init__(self) -> None:
        self.robotId = p.loadURDF("body.urdf")
        pyrosim.Prepare_To_Simulate(self.robotId)

        self.Prepare_To_Sense()
        self.Prepare_To_Act()

    def Prepare_To_Sense(self):
        self.sensors = {}

        for linkName in pyrosim.linkNamesToIndices:
            self.sensors[linkName] = SENSOR(linkName)

    def Sense(self, step):
        for sensorName, sensor in self.sensors.items():
            sensor.Get_Value(step)

    def Prepare_To_Act(self):
        self.motors = {}

        for jointName in pyrosim.jointNamesToIndices:
            self.motors[jointName] = MOTOR(jointName) 

    def Act(self, step):
        for jointName, motor in self.motors.items():
            motor.Set_Value(self, step)