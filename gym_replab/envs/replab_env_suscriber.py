import rospy
from controller import *
from config import *
import numpy as np

import rospy
from std_msgs.msg import String
from rospy_tutorials.msg import Floats
from rospy.numpy_msg import numpy_msg
class WidowXMover():
	def __init__():
		self.widowx = WidowX(boundaries)
		self.action_subscriber = rospy.Subscriber("/replab/action", numpy_msg(Floats), take_action)
		self.reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
		self.reset_publisher = rospy.Publisher("/replab/reset/finished", String)
		self.state_publisher
		self.val = widowx.move_to_position(action[0], action[1], action[2])

		def get_state():
			pos = widowx.get_current_pose().pose.position
			return [pos.x, pos.y, pos.z]
		def get_reward(goal):
		    """ Reward is negative L2 distance from objective, squared """
		    return -(np.linalg.norm(np.array(goal) - np.array(get_state()))**2)

		def step(action):
			current_pos = get_state()
			step = ast.literal_eval(data)

		def reset(useless_data):
			widowx.move_to_reset()
			reset_publisher.publish(String("Finished Reset"))

widowx = WidowX(boundaries)

reset_publisher = rospy.Publisher("/replab/reset/finished", String)

def get_state():
	pos = widowx.get_current_pose().pose.position
	return [pos.x, pos.y, pos.z]

def get_reward(goal):
	""" Reward is negative L2 distance from objective, squared """
	return -(np.linalg.norm(np.array(goal) - np.array(get_state()))**2)

def take_action(data):
	action = data.data
	widowx.move_to_position(action[0], action[1], action[2])

def reset(data):
	widowx.move_to_reset()
	reset_publisher.publish(String("Finished Reset"))

def take_action(action):
	current_pos = get_state()
	action = ast.literal_eval(str(action))

	goal = list(map(add, action, current_pos))

	take_action(goal)
	reward = get_reward(goal)
	ob = get_state()

	episode_over = False

	ret_obj = [ob, reward, episode_over, {}]
	return ret_obj




def listener():
	action_subscriber = rospy.Subscriber("/replab/step", String, take_action)
	reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
	rospy.init_node('replab_gym_node')
	rospy.spin()


if __name__ == "__main__":
	listener()