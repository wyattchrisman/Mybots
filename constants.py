import numpy

# Step size
size = 1000
sleep = 1/1300

# Back Leg
amplitude_back = numpy.pi / 4
frequency_back = 20
phaseOffset_back = numpy.pi / 2

# Front leg
amplitude_front = 0
frequency_front = 15
phaseOffset_front = 0

numberOfGenerations = 15
populationSize = 15

numSensorNeurons = 5
numMotorNeurons = 8

motorJointRange = 0.2
