import pyrosim.pyrosim as pyrosim
import pybullet as p
import pybullet_data
import numpy
import time
import random
import matplotlib.pyplot as plt

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.setGravity(0,0,-9.8, physicsClient)
planeId = p.loadURDF("plane.urdf")
robotId = p.loadURDF("body.urdf")
p.loadSDF("world.sdf")

pyrosim.Prepare_To_Simulate(robotId)

size = 1000
backLegSensorValues = numpy.zeros(size)
frontLegSensorValues = numpy.zeros(size)

# All angles
angles = numpy.linspace(0,2*numpy.pi,num=size)

# Back leg
amplitude_back = 0
frequency_back = 20
phaseOffset_back = 0
targetAngles_back = [amplitude_back * numpy.sin(frequency_back * i + phaseOffset_back) for i in angles]

# Front leg
amplitude_front = numpy.pi / 4
frequency_front = 15
phaseOffset_front = numpy.pi / 2
targetAngles_front = [amplitude_front * numpy.sin(frequency_front * i + phaseOffset_front) for i in angles]


outfile_back = 'data/targetAngles_back.npy'
outfile_front = 'data/targetAngles_front.npy'
numpy.save(outfile_back, targetAngles_back)
numpy.save(outfile_front, targetAngles_front)

#exit()
for step in range(size):
    p.stepSimulation()
    
    #backLegSensorValues[step] = pyrosim.Get_Touch_Sensor_Value_For_Link("BackLeg")
    #frontLegSensorValues[step] = pyrosim.Get_Touch_Sensor_Value_For_Link("FrontLeg")
    
    
    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_BackLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAngles_back[step],
        maxForce = 30
    )

    pyrosim.Set_Motor_For_Joint(
        bodyIndex = robotId,
        jointName = b'Torso_FrontLeg',
        controlMode = p.POSITION_CONTROL,
        targetPosition = targetAngles_front[step],
        maxForce = 30
    )

    time.sleep(1/240)

outfile_back = 'data/backLegSensorValues.npy'
outfile_front = 'data/frontLegSensorValues.npy'

numpy.save(outfile_back, backLegSensorValues)
numpy.save(outfile_front, frontLegSensorValues)
p.disconnect()