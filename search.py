import os

for _ in range(5):
    os.system("/usr/local/bin/python3 generate.py")
    os.system('/usr/local/bin/python3 simulate.py')