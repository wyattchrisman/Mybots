import pybullet as p

class WORLD:

    def __init__(self) -> None:
        self.planeId = p.loadURDF("plane.urdf")
        
        p.loadSDF("world.sdf")