import pyrosim.pyrosim as pyrosim
import pybullet as p
import pybullet_data
import numpy
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0,0,-9.8, physicsClient)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

size = 100
backLegSensorValues = numpy.zeros(size)
frontLegSensorValues = numpy.zeros(size)

for step in range(size):
    p.stepSimulation()
    backLegSensorValues[step] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    frontLegSensorValues[step] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    time.sleep(1/60)
    # print(step)

outfile_back = 'data/backLegSensorValues.npy'
outfile_front = 'data/frontLegSensorValues.npy'

numpy.save(outfile_back, backLegSensorValues)
numpy.save(outfile_front, frontLegSensorValues)
p.disconnect()