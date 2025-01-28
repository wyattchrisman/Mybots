import pybullet as p
import time

physicsClient = p.connect(p.GUI)
# p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)

p.loadSDF("box.sdf")

for step in range(10000):
    p.stepSimulation()
    time.sleep(1/60)
    # print(step)
p.disconnect()