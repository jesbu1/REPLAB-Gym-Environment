import gym
from gym import error, spaces, utils
from gym.utils import seeding
from numbers import Number

from collections import OrderedDict 

import numpy as np


import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
from std_msgs.msg import String

class ReplabEnv(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, boundaries=True):
        rospy.init_node("widowx_custom_controller")
        self.observation_space = spaces.Box(low=np.array([-.12, -.23, 0.14]), high=np.array([.12, .23, .41]))
        self.action_space = spaces.Box(low = np.array([-0.05, -0.05, -0.05]), high = np.array([0.05, 0.05, 0.05]))
        self.reset_publisher = rospy.Publisher("/replab/reset", String, queue_size=1)
        self.task_finished_subscriber = rospy.Subscriber("/replab/finished", String, self.task_toggle)
        self.task_finished = False
        self.position_updated_publisher = rospy.Publisher("/replab/received/position", String, queue_size=1)
        self.action_publisher = rospy.Publisher("/replab/action", numpy_msg(Floats), queue_size=1)
        self.current_position_subscriber = rospy.Subscriber("/replab/action/observation", numpy_msg(Floats), self.update_position)
        rospy.sleep(2)
        self.current_pos = None
        #Hardcoded goal for now
        self.goal = np.array([-0.091, 0.0864, 0.322])
        self.reset()
        
    def update_position(self, x):
        self.current_pos = np.array(x.data)
        self.position_updated_publisher.publish('received')
    def task_toggle(self, _):
        self.task_finished = not self.task_finished
    def wait_for_task(self):
        while self.task_finished is False:
            continue
        self.task_toggle(None)
    def sample_goal_for_rollout(self):
        return np.random.uniform(low=np.array([-.118, -.228, 0]), high=np.array([.118, .358, .434]), size=(3, ))
    def set_goal(self, goal):
        self.goal = goal
            
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
        self.task_finished = False
        self.action_publisher.publish(action)
        #self.wait_for_task()
        #self.task_toggle
        self.current_pos = np.array(rospy.wait_for_message("/replab/action/observation", numpy_msg(Floats)).data)
 
        reward = self._get_reward(self.goal)

        episode_over = False
        total_distance_from_goal = np.sqrt(-reward)
        info = {}
        info['total_distance'] = total_distance_from_goal
        return self.current_pos, reward, episode_over, info

    def reset(self):
        self.reset_publisher.publish(String("RESET"))
        self.current_pos = np.array(rospy.wait_for_message("/replab/action/observation", numpy_msg(Floats)).data)
        return self.current_pos

    def _get_reward(self, goal):
        return - (np.linalg.norm(self.current_pos - goal) ** 2)

    def render(self, mode='human', close=False):
        pass
    
    def get_diagnostics(self, paths):
        def get_stat_in_paths(paths, dict_name, scalar_name):
            if len(paths) == 0:
                return np.array([[]])

            if type(paths[0][dict_name]) == dict:
                # Support rllab interface
                return [path[dict_name][scalar_name] for path in paths]

            return [[info[scalar_name] for info in path[dict_name]] for path in paths]
        def create_stats_ordered_dict(
                name,
                data,
                stat_prefix=None,
                always_show_all_stats=True,
                exclude_max_min=False,
        ):
            if stat_prefix is not None:
                name = "{} {}".format(stat_prefix, name)
            if isinstance(data, Number):
                return OrderedDict({name: data})

            if len(data) == 0:
                return OrderedDict()

            if isinstance(data, tuple):
                ordered_dict = OrderedDict()
                for number, d in enumerate(data):
                    sub_dict = create_stats_ordered_dict(
                        "{0}_{1}".format(name, number),
                        d,
                    )
                    ordered_dict.update(sub_dict)
                return ordered_dict

            if isinstance(data, list):
                try:
                    iter(data[0])
                except TypeError:
                    pass
                else:
                    data = np.concatenate(data)

            if (isinstance(data, np.ndarray) and data.size == 1
                    and not always_show_all_stats):
                return OrderedDict({name: float(data)})

            stats = OrderedDict([
                (name + ' Mean', np.mean(data)),
                (name + ' Std', np.std(data)),
            ])
            if not exclude_max_min:
                stats[name + ' Max'] = np.max(data)
                stats[name + ' Min'] = np.min(data)
            return stats
        statistics = OrderedDict()
        stat_name = 'total_distance'
        stat = get_stat_in_paths(paths, 'env_infos', stat_name)
        statistics.update(create_stats_ordered_dict('Final %s' % (stat_name), [s[-1] for s in stat], always_show_all_stats=True,))
        return statistics

    def close(self):
        pass
