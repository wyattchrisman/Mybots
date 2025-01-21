import pybullet as p
import time

physicsClient = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
for step in range(1000):
    p.stepSimulation()
    time.sleep(1/60)
    print(step)
p.disconnect()