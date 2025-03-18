import os
from hillclimber import HILL_CLIMBER


hc = HILL_CLIMBER()

hc.Evolve()

hc.Show_Best()



'''
for _ in range(5):
    os.system("/usr/local/bin/python3 generate.py")
    os.system('/usr/local/bin/python3 simulate.py')
'''