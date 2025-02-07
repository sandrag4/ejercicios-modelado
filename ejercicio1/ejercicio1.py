import pybullet as p
import pybullet_data
import time

physicsClient = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

planeId = p.loadURDF("plane.urdf")

# angulos iniciales
euler_angles = [0,0,0]
startOrientation = p.getQuaternionFromEuler(euler_angles)

# posicion inicial
startPosition = [0,0,0.2]

robotId = p.loadURDF("ejercicio1.urdf", startPosition, startOrientation)

# joint que se va a mover
joint2 = 1

p.setJointMotorControl2(robotId, joint2, controlMode=p.VELOCITY_CONTROL, force=0)

# sliders de friccion y torque
friccion_id = p.addUserDebugParameter("friccion", 0.0, 5.0, 0.1)
torque_id = p.addUserDebugParameter("torque", -10.0, 10.0, 0.0)

while (1):
    # valores de los sliders
    friccion = p.readUserDebugParameter(friccion_id)
    torque = p.readUserDebugParameter(torque_id)

    # aplicar la friccion y torque
    p.changeDynamics(robotId, joint2, jointDamping=friccion)
    p.setJointMotorControl2(robotId, joint2, controlMode=p.TORQUE_CONTROL, force=torque)

    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
