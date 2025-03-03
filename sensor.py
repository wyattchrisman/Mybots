import pyrosim.pyrosim as pyrosim
from typing import Any
import constants as c
import numpy

class SENSOR:

    def __init__(self, linkName) -> None:
        self.linkName = linkName
        self.values = numpy.zeros(c.size)

    def Get_Value(self, step):
        self.values[step] = pyrosim.Get_Touch_Sensor_Value_For_Link(self.linkName)
        
        if step == c.size - 1:
            print(self.linkName, self.values)
        
    def Save_Values(self):
        filename = f"data/{self.linkName}_sensor.npy"
        numpy.save(filename, self.values)