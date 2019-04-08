import rospy
from controller import *
from config import *
import numpy as np

import rospy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg

widowx = None
def activate_widowx():
    global widowx
    widowx = WidowX(True)

task_finished = False
def task_toggle(_):
    global task_finished
    task_finished = not task_finished
def wait_for_task():
    while task_finished is False:
        continue
    task_toggle(None)
    
    
#finished_publisher = rospy.Publisher("/replab/finished", String, queue_size=1)
observation_publisher = rospy.Publisher("/replab/action/observation", numpy_msg(Floats), queue_size=1)
position_updated_subscriber = rospy.Subscriber("/replab/received/position", String, task_toggle)

rospy.sleep(2)
def get_state():
	pos = widowx.get_current_pose().pose.position
	return [pos.x, pos.y, pos.z]

def get_reward(goal):
	""" Reward is negative L2 distance from objective, squared """
	return -(np.linalg.norm(np.array(goal) - np.array(get_state()))**2)

def take_action(data):
	"""
	Publishes [current x, current y, current z]
	"""
	action = data.data
	current_pos = np.array(get_state(), dtype=np.float32)
	print("Current Pos: " + str(current_pos))
	goal = np.add(action, current_pos)
	print("Action: " + str(action))
	print("Where I'm going: " + str(goal))
	widowx.move_to_position(float(goal[0]), float(goal[1]), float(goal[2]))
	current_state = np.array(get_state(), dtype=np.float32)
	task_finished = False
	observation_publisher.publish(current_state)  
	#wait_for_task()
	#finished_publisher.publish(String("Finished"))
	#observation_publisher.publish(np.array(get_state(), dtype=np.float32))

def reset(data):
	widowx.move_to_reset()
	rospy.sleep(0.5)
	observation_publisher.publish(np.array(get_state(), dtype=np.float32))
	#wait_for_task()
	#finished_publisher.publish(String("Finished"))

def listener():
	action_subscriber = rospy.Subscriber("/replab/action", numpy_msg(Floats), take_action)
	reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
	activate_widowx()
	rospy.spin()

if __name__ == "__main__":
    rospy.init_node('replab_gym_node')
    listener()

