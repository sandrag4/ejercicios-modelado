import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,0)

planeId = p.loadURDF("plane.urdf")

x_pos_id = 0
y_pos_id = 0
z_pos_id = 3

v = 0
g = -9.8
t = 1./240.

euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)
startPosition = [x_pos_id,y_pos_id,z_pos_id]

robotId = p.loadURDF("esfera.urdf", startPosition, startOrientation)

while (1):

    if (z_pos_id > 0.5):
        v = v + g * t
        z_pos_id = z_pos_id + v * t + (1/2) * g * t**2

    actualPosition = [x_pos_id,y_pos_id,z_pos_id]
    p.resetBasePositionAndOrientation(robotId, actualPosition, startOrientation)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
