import pyrosim.pyrosim as pyrosim

pyrosim.Start_SDF("boxes.sdf")

length = 1
width = 1
height = 1

x = 0
y = 0 
z = 0.5

size = 5

for i in range(size):
    x = 0
    for j in range(size):
        for k in range(10):
            decrease_size = 0.9 ** k
            pyrosim.Send_Cube(name="Box", pos=[x,y,z+k], size=[length*decrease_size, width*decrease_size, height*decrease_size])

        x += 1

    y += 1

pyrosim.End()