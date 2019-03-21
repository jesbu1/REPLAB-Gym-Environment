import gym
from gym import error, spaces, utils
from gym.utils import seeding



import numpy as np


import rospy
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float32
from std_msgs.msg import String

class ReplabEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, boundaries=True):
        rospy.init_node("widowx_custom_controller")
        self.observation_space = spaces.Box(low=np.array([-.118, -.228, 0]), high=np.array([.118, .358, .434]))
        self.action_space = spaces.Box(low = np.array([-0.01, -0.01, -0.01]), high = np.array([0.01, 0.01, 0.01]))
        self.reset_publisher = rospy.Publisher("/replab/reset", String, queue_size=1)
        self.task_finished_subscriber = rospy.Subscriber("/replab/finished", String, self.task_toggle)
        self.task_finished = False
        
        self.action_publisher = rospy.Publisher("/replab/action", numpy_msg(Float32), queue_size=1)
        rospy.sleep(2)
        self.current_position_subscriber = rospy.Subscriber("/replab/action/observation", numpy_msg(Float32), self.update_position)

        self.current_pos = np.array([0.015339807878856412, -1.2931458041875956, 1.0109710760673565], dtype=np.float32)
        #Hardcoded goal for now
        self.goal = np.array([-0.042, 0.5, 0.05])
        self.reset()
    def update_position(self, x):
        self.current_pos = np.array(x.data)
    def task_toggle(self, _):
        self.task_finished = not self.task_finished
    def wait_for_task(self):
        while self.task_finished is False:
            continue
        self.task_toggle(None)
            
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
        self.wait_for_task()
        self.task_toggle

        reward = self._get_reward(self.goal)

        episode_over = False
        return self.current_pos, reward, episode_over, {}

    def reset(self):
        self.reset_publisher.publish(String("RESET"))
        self.wait_for_task()
        return

    def _get_reward(self, goal):
        return - (np.linalg.norm(self.current_pos - goal) ** 2)

    def render(self, mode='human', close=False):
        pass

    def close(self):
        pass
