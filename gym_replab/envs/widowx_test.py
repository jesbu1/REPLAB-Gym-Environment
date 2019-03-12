import pybullet as p
import time

p.connect(p.GUI)
#path = "/Volumes/64GB/RAIL/bullet3/data/"
#path = '../../..//bullet3/data/'
path = "./"
p.loadURDF(path + "table/table.urdf",[0, 0, 0],[0.000000,0.000000,0.0,1])
p.setGravity(0,0,-10)
arm = p.loadURDF(path + "/widowx/widowx.urdf",useFixedBase=True)

p.resetBasePositionAndOrientation(arm,[0,0,0.58],[0.000000,0.000000,0.000000,-1.000000])
  
#cuid = p.createCollisionShape(p.GEOM_BOX, halfExtents = [0.5, 0.5, 0.055])
#mass= 0 #static box
#p.createMultiBody(mass,cuid)
#p.resetBasePositionAndOrientation(cuid, [0, 0, -0.194018 - 0.055],[0, 0, 0, 0])
while (1):
	p.stepSimulation()
	time.sleep(0.01)
	#p.saveWorld("test.py")
	viewMat = p.getDebugVisualizerCamera()[2]
	#projMatrix = [0.7499999403953552, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.0000200271606445, -1.0, 0.0, 0.0, -0.02000020071864128, 0.0]
	projMatrix = p.getDebugVisualizerCamera()[3]
	width=640
	height=480
	img_arr = p.getCameraImage(width=width,height=height,viewMatrix=viewMat,projectionMatrix=projMatrix)
