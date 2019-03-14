import rospy
from controller import *
from config import *
from std_msgs.msg import String
import ast
import numpy as np
class WidowXMover():
	def __init__():
		self.widowx = WidowX(boundaries)
		self.action_subscriber = rospy.Subscriber("/replab/step", String, take_action)
		self.reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
		self.reset_publisher = rospy.Publisher("/replab/reset/finished", String)
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
			reset_publisher.publish("DONE")


		rospy.spin()