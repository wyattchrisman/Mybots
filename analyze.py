import matplotlib.pyplot as plt
import numpy

outfile_back = 'data/backLegSensorValues.npy'
outfile_front = 'data/frontLegSensorValues.npy'

backLegSensorValues = numpy.load(outfile_back)
frontLegSensorValues = numpy.load(outfile_front)

plt.plot(backLegSensorValues, label="Back Leg", linewidth=3)
plt.plot(frontLegSensorValues, label="Front Leg")

plt.legend()

plt.show()