#! /usr/bin/env python3

import random
from openai_ros.openai_ros_common import StartOpenAI_ROS_Environment
import rospy
import torch
import gym
import os
from her.ddpg_her_normalization import *
from her.normalizer import normalizer
import numpy as np
from mpi4py import MPI
import time
from openai_ros.task_envs.nachi import nachi_random_world
from openai_ros.robot_envs import nachi_env
from openai_ros import robot_gazebo_env
from Real_robot.Nachi_Comm import Socket_comm
from grasp import Gripper
import pandas as pd
class Hyper_Params:
    def __init__(self):
        self.env_name = "NachiReach_v0"
        self.seed = 1506

        # number of epochs for training
        self.num_epochs = 100

        # number of episodes - the times to collect samplers per epoch (reset -> new goal ->action)
        self.num_episodes = 50

        # maximum step for 1 episode
        self.max_ep_step = 300  # steps

        # the times to update networks
        self.num_batches = 50  # (divide the dataset into 50 batch, shuffle and random sample for each batch)

        # batch size
        self.batch_size = 300

        # initial number of step for random exploration
        # self.start_steps = 10000

        # size of replay buffer
        self.buff_size = 1000000  #  buffer size => 1000000 transitions

        # test phase
        self.phase = "test"

        # path to save model
        self.save_dir = (
            "/home/qind/Desktop/catkin_ws/src/openai_hac/scripts/her/saved_models"
        )

        # number of episodes testing should run
        self.test_episodes = 100

        # the clip ratio
        self.clip_obs = np.inf

        # the clip range
        self.clip_range = np.inf

        # learning rate actor
        self.lr_actor = 0.001

        # learning rate critic
        self.lr_critic = 0.001

        # scaling factor for gausian noise on action
        self.noise_eps = 0.1

        # random epsilon
        self.random_eps = 0.3

        # discount factor in bellman equation
        self.gamma = 0.98

        # polyak value for averaging
        self.polyak = 0.95

        # cuda - using GPU?
        self.cuda = True

        # number of worker (load data from cpu to gpu)
        # self.num_workers = 1

        # the rollout per MPI
        self.num_rollouts_per_mpi = 2

        # l2 regularization
        self.action_l2 = 1

        # replay_k
        self.replay_k = 4

        # threshold success
        self.threshold = 0.005

        # training space
        self.position_x_max = 0.63
        self.position_x_min = 0.3
        self.position_y_max = 0.145
        self.position_y_min = -0.145
        self.position_z_max = 0.31
        self.position_z_min = 0.15

class Real_Move():
    def __init__(self, params, env, env_params, robot, grip, arr1, arr2) -> None:
        self.env = env
        self.env_params = env_params
        self.params = params
        self.robot = robot
        self.grip = grip
        # load actor model
        self.actor = Actor(self.env_params)
        self.position_ee_x_max = rospy.get_param("/HER/position_ee_x_max")*1000
        self.position_ee_x_min = rospy.get_param("/HER/position_ee_x_min")*1000
        self.position_ee_y_max = rospy.get_param("/HER/position_ee_y_max")*1000
        self.position_ee_y_min = rospy.get_param("/HER/position_ee_y_min")*1000
        self.position_ee_z_max = rospy.get_param("/HER/position_ee_z_max")*1000
        self.position_ee_z_min = rospy.get_param("/HER/position_ee_z_min")*1000
        self.rot = np.array([-177.3, 90, -177.3])
        self.arr1 = arr1
        self.arr2 = arr2
        # create the normalizer
        self.o_norm = normalizer(
                size=self.env_params["obs_dim"], default_clip_range=self.params.clip_range
            )
        self.g_norm = normalizer(
                size=self.env_params["goal_dim"], default_clip_range=self.params.clip_range
            )   
        if MPI.COMM_WORLD.Get_rank() == 0:
            if os.path.exists(
                os.path.join(
                    self.params.save_dir, self.params.env_name, "actor_critic.pt"
                )
            ):
                checkpoint = torch.load(
                    os.path.join(
                        self.params.save_dir, self.params.env_name, "actor_critic.pt"
                    )
                )
                self.actor.load_state_dict(checkpoint["actor_state_dict"])

                self.actor.eval()
                self.g_norm.mean = checkpoint['g_mean']
                self.o_norm.mean = checkpoint['o_mean']
                self.g_norm.std = checkpoint['g_std']
                self.o_norm.std = checkpoint['o_std']
        
        if self.params.cuda and torch.cuda.is_available():
            device = torch.device("cuda")
        else:
            device = torch.device("cpu")

        self.actor.to(device)

    def _preproc_inputs(self,o, g):
        obs_norm = self.o_norm.normalize(o)
        g_norm = self.g_norm.normalize(g)

        inputs = np.concatenate([obs_norm, g_norm])

        inputs = torch.tensor(inputs, dtype=torch.float32).unsqueeze(0)
        if params.cuda:
            inputs = inputs.cuda()
        return inputs

    def remap(self, x, oMin, oMax, nMin, nMax):

        # range check
        if oMin == oMax:
            print("Warning: Zero input range")
            return None

        if nMin == nMax:
            print("Warning: Zero output range")
            return None

        # check reversed input range
        reverseInput = False
        oldMin = min(oMin, oMax)
        oldMax = max(oMin, oMax)
        if not oldMin == oMin:
            reverseInput = True

        # check reversed output range
        reverseOutput = False
        newMin = min(nMin, nMax)
        newMax = max(nMin, nMax)
        if not newMin == nMin:
            reverseOutput = True

        portion = (x - oldMin) * (newMax - newMin) / (oldMax - oldMin)
        if reverseInput:
            portion = (oldMax - x) * (newMax - newMin) / (oldMax - oldMin)

        result = portion + newMin
        if reverseOutput:
            result = newMax - portion

        return result

    def set_trajectory_ee(self, action):
        """
        Sets the Pose of the EndEffector based on the action variable.
        The action variable contains the position and orientation of the EndEffector.
        See create_action
        """
        # Set up a trajectory message to publish.
        ee_target = [None] * 6
        ee_target[0] = action[0]
        ee_target[1] = action[1]
        ee_target[2] = action[2]

        # orientation in rpy
        ee_target[3] = action[3]
        ee_target[4] = action[4]
        ee_target[5] = action[5]

        # clip the action values to forcefully have them fall into a safe range for the execution.
        ee_target = self.trajectory_processing(ee_target)
        rospy.logwarn("============== Action: {0}".format(ee_target))
        # set action for real-robot here
        self.robot.moveposition(ee_target, 'machine_abs_linear')

    def trajectory_processing(self, ee_target):
        
        _pose = list(map(float,self.robot.tool_coordinate()[:6]))
        x = _pose[0]
        y = _pose[1]
        z = _pose[2]
        arr = [x,y,z]
        self.arr2 = np.append(self.arr2, arr)
        ee_target[0] = np.clip(
            x + ee_target[0], self.position_ee_x_min, self.position_ee_x_max
        )
        ee_target[1] = np.clip(
            y + ee_target[1], self.position_ee_y_min, self.position_ee_y_max
        )
        ee_target[2] = np.clip(
            z + ee_target[2], self.position_ee_z_min, self.position_ee_z_max
        )
        print("XYZ absolute: {0}".format(ee_target))
        return ee_target


    def move_HER_IK(self, x, y, z, a):

        # set goal data (input)
        g = np.array([x, y, z])
        self.env.set_goal(x, y, z, 0, 0, 0, 1.0)
        obs = self.env.get_obs()
        print(obs["desired_goal"])
        o = obs['observation']
        self.arr1 = np.append(self.arr1,o)
        print(o[:3]*1000)
        g = obs['desired_goal']
        Done = False
        step = 0
        while not (Done or (step == self.params.max_ep_step)):
                
                with torch.no_grad():

                    # pre-process input
                    input_tensor = self._preproc_inputs(o, g)

                    #feed to actor
                    pi = self.actor(input_tensor)

                    # convert the actions
                    actions = pi.detach().cpu().numpy().squeeze()

                print(step)    

                # set action in virtual env
                observation_new, _, Done, _ = self.env.step(actions)
                time.sleep(0.5)
                actions = np.around(self.remap(actions[:3], -1, 1, -0.01, 0.01),4)
                actions = actions*1000
                print("Delta position: {0}".format(actions))          #delta position
                
                action = np.concatenate([actions, self.rot])

                #Move Real Robot
                self.set_trajectory_ee(action.tolist())

                # get new observation
                o = observation_new["observation"]
                print("New obs: {0}".format(o[:3]*1000))
                self.arr1 = np.append(self.arr1,o)
                step += 1
                print('>>>>>>>>>>>>>>>>>>>>>><<<<<<<<<<<<<<<<<<<<<<<<<<<<')
        rospy.logwarn(
            "####################### Complete Move ##########################"
        )
        print(">>>>>> Complete in {0} steps <<<<<<<".format(step))
        if a == 1:
            self.grip.Grasp()
        elif a == 0:
            self.grip.Drop()
        else:
            pass

    

if __name__ == "__main__":
    
    os.environ["OMP_NUM_THREADS"] = "1"
    os.environ["MKL_NUM_THREADS"] = "1"
    os.environ["IN_MPI"] = "1"

    params = Hyper_Params()
    rospy.init_node("HER_reach")
    
    # init virtual env
    task_and_robot_environment_name = rospy.get_param(
        "/Nachi/task_and_robot_environment_name"
    )

    env = StartOpenAI_ROS_Environment(
        task_and_robot_environment_name, params.max_ep_step
    )

    # set seed - make sure model give same result every run
    env.seed(params.seed + MPI.COMM_WORLD.Get_rank())
    env.action_space.seed(params.seed + MPI.COMM_WORLD.Get_rank())
    torch.manual_seed(params.seed + MPI.COMM_WORLD.Get_rank())
    np.random.seed(params.seed + MPI.COMM_WORLD.Get_rank())
    random.seed(params.seed + MPI.COMM_WORLD.Get_rank())
    
    if params.cuda:
        torch.cuda.manual_seed(params.seed + MPI.COMM_WORLD.Get_rank())
    
    # connect to real robot TODO:
    Robot = Socket_comm()
    Robot.socket_initalize()

    #Initialize Gripper
    grip = Gripper()
    # reset and get observation (vir robot)
    obs = env.reset()

    goal = obs["desired_goal"]
    # move home real robot TODO:
    Robot.move_home()

    print("Home Position Joint {0}".format(Robot.joint_coordinate()))
    print("Position Tool {0}".format(Robot.tool_coordinate()))

    print("initial random goal is: {0}".format(goal))
    env_params = {
        "obs_dim": obs["observation"].shape[0],
        "goal_dim": obs["desired_goal"].shape[0],
        "action_dim": env.action_space.shape[0],
        "action_max": env.action_space.high[0],
        "max_timesteps": env._max_episode_steps,  # max_step for each ep
    }
    arr1 = np.array([])
    arr2 = np.array([])
    ddpg_agent = Real_Move(params, env, env_params, Robot, grip, arr1, arr2)
    
   
    x = input('Complete Detection. Start moving ?(Y/N)')
    ddpg_agent.move_HER_IK(0.43,0.14,0.3, 0.5)
    ddpg_agent.move_HER_IK(0.43,0.14,0.22, 1)            # coordinate pick 0.45 0.11 0.2
    ddpg_agent.move_HER_IK(0.4, -0.11, 0.3, 0.5)       # coordinate bridge 0.35 -0.11 0.3
    ddpg_agent.move_HER_IK(0.38, -0.144, 0.26, 0)      # Coordinate place 0.35 -0.145 0.225 
    env.reset()
    time.sleep(0.5)
    Robot.move_home()
    time.sleep(10)
    x = input('Complete Detection. Start moving ?(Y/N)')
    ddpg_agent.move_HER_IK(0.532, 0.026, 0.32, 1)
    ddpg_agent.move_HER_IK(0.38, -0.144, 0.3, 0.5)       # coordinate bridge 0.35 -0.11 0.3
    ddpg_agent.move_HER_IK(0.38, -0.144, 0.24, 0)      # Coordinate place 0.35 -0.145 0.225 
    env.reset()
    time.sleep(0.5)
    Robot.move_home()
    #export array to CSV file (using 2 decimal places)
    np.savetxt("/home/qind/Desktop/real_robot.csv", arr2, delimiter=",", fmt="%.2f", \
                    header="X, Y, Z", comments="")
    np.savetxt("/home/qind/Desktop/virtual_robot.csv", arr1, delimiter=",", fmt="%.6f", \
                    header="X, Y, Z, d, rel_pos", comments="")
    time.sleep(10)
