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
        self.widowx = WidowX(boundaries)
        self.widowx.move_to_reset()
        self.reset_publisher = rospy.Publisher("/replab/reset", String, queue_size=1)
        self.reset_subscriber = rospy.Subscriber("/replab/reset/finished", String, reset_checker)
        self.action_publisher = rospy.Publisher("/replab/action", numpy_msg(Floats), queue_size=1)
        rospy.init_node('replab_gym_node')
        
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
        self.action_publisher.publish(action)
        rospy.wait_for_message("/replab/action/finished", String)

        current_pos = self._get_state()

        goal = list(map(add, action, current_pos))

        self._take_action(goal)

        reward = self._get_reward(goal)
        ob = self._get_state()

        episode_over = False
        return ob, reward, episode_over, {}

    def reset(self):
        self.reset_publisher.publish(String("RESET"))
        rospy.wait_for_message("/replab/reset/finished", String)
        return


    def render(self, mode='human', close=False):
        pass

    def close(self):
        pass

    def _take_action(self, action):
        val = self.widowx.move_to_position(action[0], action[1], action[2])
        if val == False:
            #Couldn't move
            pass
        
    def _get_goal_status(self):
        pass

    def _get_reward(self, state, goal):
        """ Reward is negative L2 distance from objective, squared """
        
        return - (np.linalg.norm(np.array(goal) - np.array(self._get_state()))**2)

    def _get_state(self):
        """
        get_current_pose returns obj with pose(with acctributes posiiton, orientation)
        """
        pos = self.widowx.get_current_pose().pose.position
        return [pos.x, pos.y, pos.z]
