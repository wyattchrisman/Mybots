import os
from parallelHC import PARALLEL_HILL_CLIMBER


phc = PARALLEL_HILL_CLIMBER()

phc.Evolve()

phc.Show_Best()

'''
for _ in range(5):
    os.system("/usr/local/bin/python3 generate.py")
    os.system('/usr/local/bin/python3 simulate.py')
'''