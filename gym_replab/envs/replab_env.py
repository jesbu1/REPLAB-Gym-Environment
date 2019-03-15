import gym
from gym import error, spaces, utils
from gym.utils import seeding



from controller import *
import numpy as np
from config import *

import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String

class ReplabEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, boundaries=True):
        rospy.init_node("widowx-custom_controller")

        self.reset_publisher = rospy.Publisher("/replab/reset", String, queue_size=1)
        self.reset_subscriber = rospy.Subscriber("/replab/reset/finished", String, reset_checker)
        self.action_publisher = rospy.Publisher("/replab/action", numpy_msg(Floats), queue_size=1)
        rospy.init_node('replab_gym_node')
        self.current_pos = np.array([0.015339807878856412, -1.2931458041875956, 1.0109710760673565], dtype=np.float32)
        #Hardcoded goal for now
        self.goal = np.array([-0.042, 0.5, 0.05])
        self.reset()
        
    def step(self, action):
        """

        Parameters
        ----------
        action : [change in x, change in y, change in z]

        Returns
        -------
        ob, reward, episode_over, info : tuple
            ob (object) :
                an environment-specific object representing your observation of
                the environment.
            reward (float) :
                amount of reward achieved by the previous action. The scale
                varies between environments, but the goal is always to increase
                your total reward.
            episode_over (bool) :
                whether it's time to reset the environment again. Most (but not
                all) tasks are divided up into well-defined episodes, and done
                being True indicates the episode has terminated. (For example,
                perhaps the pole tipped too far, or you lost your last life.)
            info (dict) :
                 diagnostic information useful for debugging. It can sometimes
                 be useful for learning (for example, it might contain the raw
                 probabilities behind the environment's last state change).
                 However, official evaluations of your agent are not allowed to
                 use this for learning.
        """
        action = np.array(action, dtype=np.float32)
        target_pos = np.add(action, self.current_pos)

        self.action_publisher.publish(action)
        self.current_pos = rospy.wait_for_message("/replab/action/observation", numpy_msg(Floats)).data

        reward = self._get_reward(self.goal)

        episode_over = False
        return self.current_pos, reward, episode_over, {}

    def reset(self):
        self.reset_publisher.publish(String("RESET"))
        rospy.wait_for_message("/replab/reset/finished", String)
        return

    def _get_reward(self, goal):
        return - (np.linalg.norm(self.current_pos - goal) ** 2)

    def render(self, mode='human', close=False):
        pass

    def close(self):
        pass
