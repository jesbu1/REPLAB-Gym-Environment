import pybullet as p
import time
import math
import random
import numpy as np
p.connect(p.GUI)
offset = np.array([-0.185033226409,0.00128528,0.46227163])
path = './'
arm = p.loadURDF(path + "/URDFs/widowx/widowx.urdf", useFixedBase=True)
NEUTRAL_VALUES = [0.015339807878856412, -1.4839419194602816,
                  1.4971652489763858, -0.008369006790373335, -0.08692557798018634]
p.resetBasePositionAndOrientation(arm,[0, 0, 0], p.getQuaternionFromEuler([math.pi, 0, 0]))
for i in range(p.getNumJoints(arm)):
    print(p.getJointInfo(arm, i))
#for i in range(100):
#    print(i, p.getLinkState(arm, i))
#for i in range(len(NEUTRAL_VALUES)):
#1    p.setJointMotorControl2(arm, i, controlMode=p.POSITION_CONTROL, targetPosition=NEUTRAL_VALUES[i])
current_pos = np.array(list(p.getLinkState(arm, 5, computeForwardKinematics=1)[4]))
#current_pos += offset
print(current_pos, p.getLinkState(arm, 5, computeForwardKinematics=1)[2])
while (1):
        #for i in range(100):
        #    p.stepSimulation()
        time.sleep(2)
        current_pos = np.array(list(p.getLinkState(arm, 5, computeForwardKinematics=1)[4])) + offset
        print(current_pos)
        current_pos = [current_pos[i] if i != 2 else current_pos[i] - 0.1 for i in range(len(current_pos))]
        print(current_pos)
        #new_pos = [i + random.randint(-9, 9) for i in current_pos]
        joint_pos = p.calculateInverseKinematics(arm, 5, current_pos - offset)
        #print(joint_pos)
        for i in range(5):
            p.resetJointState(
                    arm, 
                    i, 
                    joint_pos[i]
                    )
        #joint_pos = random.randint(3, 30)/1000
        #p.setJointMotorControl2(arm, 7, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos)
        #p.setJointMotorControl2(arm, 8, controlMode=p.POSITION_CONTROL, targetPosition=joint_pos)
        #p.resetJointState(arm, 7, targetValue=joint_pos)
        #p.resetJointState(arm, 8, targetValue=joint_pos)
