import rospy
from controller import *
from config import *
from std_msgs.msg import String
import ast
import numpy as np
class WidowXMover():
    def __init__():
        self.widowx = WidowX(boundaries)
        self.action_subscriber = rospy.Subscriber("/replab/step", String, step)
        self.reset_subscriber = rospy.Subscriber("/replab/reset", String, reset)
        self.reset_publisher = rospy.Publisher("/replab/reset/finished", String)
        self.reset("")

        def get_state(self,):
            pos = widowx.get_current_pose().pose.position
            return [pos.x, pos.y, pos.z]
        def _get_reward(self, goal):
            """ Reward is negative L2 distance from objective, squared """
            return -(np.linalg.norm(np.array(goal) - np.array(get_state()))**2)

        def step(self, action):
            current_pos = get_state()
            action = ast.literal_eval(data)
            goal = list(map(add, action, current_pos))
            self._take_action(goal)
            reward = self._get_reward(goal)
            ob = self._get_state()
            episode_over = False
            return ob, reward, episode_over, {}

        def reset(self, useless_data):
            widowx.move_to_reset()
            reset_publisher.publish("DONE")

        def _take_action(self, action):
            val = self.widowx.move_to_position(action[0], action[1], action[2])
            if val == False:
                #Couldn't move
                pass

        rospy.spin()