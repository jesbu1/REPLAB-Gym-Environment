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



finished_publisher = rospy.Publisher("/replab/finished", String, queue_size=1)
observation_publisher = rospy.Publisher("/replab/action/observation", numpy_msg(Floats), queue_size=1)
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
	goal = np.add(action, current_pos)
	widowx.move_to_position(float(action[0]), float(action[1]), float(action[2]))
	current_state = np.array(get_state(), dtype=np.float32)
	observation_publisher.publish(current_state)
	finished_publisher.publish(String("Finished"))

def reset(data):
	widowx.move_to_reset()
	finished_publisher.publish(String("Finished"))

def listener():
	action_subscriber = rospy.Subscriber("/replab/action", numpy_msg(Floats), take_action)
	reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
	activate_widowx()
	rospy.spin()

if __name__ == "__main__":
    rospy.init_node('replab_gym_node')
    listener()

