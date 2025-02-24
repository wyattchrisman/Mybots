import matplotlib.pyplot as plt
import numpy

'''
outfile_back = 'data/backLegSensorValues.npy'
outfile_front = 'data/frontLegSensorValues.npy'

backLegSensorValues = numpy.load(outfile_back)
frontLegSensorValues = numpy.load(outfile_front)

plt.plot(backLegSensorValues, label="Back Leg", linewidth=3)
plt.plot(frontLegSensorValues, label="Front Leg")
'''
infile_back = 'data/targetAngles_back.npy'
infile_front = 'data/targetAngles_front.npy'

sin_positions_back = numpy.load(infile_back)
sin_positions_front = numpy.load(infile_front)

plt.plot(sin_positions_front, label="Front Leg", linewidth=3)
plt.plot(sin_positions_back, label="Back Leg")

plt.legend()

plt.show()