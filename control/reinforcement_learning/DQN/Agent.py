import os
import configparser
import ast
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '3'

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib import cm
import seaborn as sns

import copy
import time
from datetime import datetime
import tensorflow as tf
from tensorflow.keras import backend as K
from tensorflow.keras.callbacks import TensorBoard

from control.reinforcement_learning.DQN.DeepQNetwork import DeepQNetwork
from control.reinforcement_learning.DQN.ReplayBuffer import ReplayBuffer

class Agent:
    """
    DQN Agent
    - Take an environment
    - Set up the deep neural network
    - Store the experience
    - Choose action
    - Train the network
    - Evaluate the network
    """
    def __init__(self, env):

        # check if gpu is available
        if tf.config.list_physical_devices('GPU'):
            # print the device name
            print("GPU is available")
            print("Device name: {}".format(tf.test.gpu_device_name()))
                
        else:
            print("GPU is not available")

        self.env = env
        
        self.nJoint = self.env.nbJoint
        
        # read INI file
        # get the path of the root directory
        root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        ini_file_path = os.path.join(root_dir, 'config.ini')
        self.params = self.parse_ini(ini_file_path)
        
        # set up the parameters from the INI file
        self.action_steps = int(self.params['action_steps'])
        self.torque_range = ast.literal_eval(self.params['control_range'])
        self.max_episode_steps = int(self.params['max_episode_steps'])
        self.train_episodes = int(self.params['train_episodes'])
        self.lr = float(self.params['lr'])
        self.discount_factor = float(self.params['discount_factor'])
        self.epsilon = float(self.params['epsilon'])
        self.epsilon_decay_episodes = int(self.params['epsilon_decay_episodes'])
        self.epsilon_final = float(self.params['epsilon_final'])
        self.buffer_size = int(self.params['buffer_size'])
        self.batch_size = int(self.params['batch_size'])
        self.hidden_dims = ast.literal_eval(self.params['hidden_dims'])
        self.update_rate_episodes = int(self.params['target_update_episodes'])
        self.train_rate_steps = int(self.params['train_rate_steps'])

        self.discounted_reward = 0.0
        self.epsilon_decay = (self.epsilon - self.epsilon_final) / self.epsilon_decay_episodes

        # set up the environment parameters
        self.env.num_actions = self.action_steps
        self.env.range_actions = self.torque_range
        self.env.maxIter = self.max_episode_steps
        self.env.umax = self.torque_range[1]
        self.env.actions = np.linspace(self.env.range_actions[0], self.env.range_actions[1], self.action_steps)
        self.env.action_space = [i for i in range(self.action_steps)]
        self.action_space = self.env.action_space

        self.total_step_counter = 0
        self.replay_buffer = ReplayBuffer(self.buffer_size)

        self.name_model = self.env.name + '_'+str(self.action_steps)+'_'+str(self.max_episode_steps)+'_' + \
                    str(self.epsilon_decay_episodes)+'_'+str(self.buffer_size)+'_'+str(self.batch_size)+'_'+ \
                    str(self.hidden_dims)+'_'+str(self.update_rate_episodes)+'_'+str(self.train_rate_steps)
        
        # path of the weights folder
        self.weights_folder = os.path.join(root_dir, 'saved_weights')
        self.final_weights_folder = os.path.join(root_dir, 'final_results/'+self.env.name)
        self.weights_name = ['dqn_weights_' + self.name_model +'.h5',
                             'dqn_target_weights_' + self.name_model +'.h5']
        
        # save metrics in a csv file (not used)
        self.metrics_folder = os.path.join(root_dir, 'saved_metrics')
        self.metrics_df = pd.DataFrame()
        self.metrics_name = ''

        # save the logs in a tensorboard file
        self.log_dir = os.path.join(root_dir, 'logs')
        self.writer = tf.summary.create_file_writer(os.path.join(self.log_dir, self.name_model))

        # create the deep neural network
        self.q_net = DeepQNetwork(self.lr, self.env.num_actions, self.env.num_state, self.hidden_dims , opt='adam', loss='mse')
        self.q_target_net = DeepQNetwork(self.lr, self.env.num_actions, self.env.num_state, self.hidden_dims, opt='adam', loss='mse')

        self.loss = 0.0
        self.current_episode = 0
        self.training_time = 0
    
    def policy(self, observation, type='epsilon_greedy'):
        """
        Choose an action based on the policy
        """
        if type == 'epsilon_greedy':
            if np.random.random() < self.epsilon:
                action = np.random.choice(self.action_space)
            else:
                action = np.argmax(self.q_net.predict(np.array([observation])))
        elif type == 'greedy':
            action = np.argmax(self.q_net.predict(np.array([observation])))
        elif type == 'random':
            action = np.random.choice(self.action_space)
        else:
            raise Exception("Unknown policy type")
        
        return action
    
    def train(self):
        """
        Train the network
        """
        # check if the replay buffer has enough experiences
        if len(self.replay_buffer.gameplay_experiences) < self.batch_size:
            return
        
        # sample a batch of experiences
        states, actions, rewards, new_states, dones = self.replay_buffer.sample_batch(self.batch_size)
        
        # predict the q values of current states and next states
        q_predicted = self.q_net.predict(states)
        q_next = self.q_target_net.predict(new_states)
        # get the maximum q value of the next states
        q_max_next = np.max(q_next, axis=1)
        # copy the q values of the current states
        q_target = q_predicted.copy()
        
        for i in range(self.batch_size):
            # Q(s, a) = r + γ * max(Q(s', a')) * (1 - done)
            # if the next state is terminal, then the q value is just the reward
            # otherwise, estimate the q value using the target network
            q_target[i, actions[i]] = rewards[i] + self.discount_factor * q_max_next[i] * (1 - dones[i])
        
        # train the network in batches
        self.loss = self.q_net.train_on_batch(states, q_target)
        # self.loss = self.q_net.train_batch_gradientTape(states, q_target)
   
    def train_model(self, render=True, plot=True, verbose=False, soft_start=False):
        """
        Train the model for a number of episodes and plot the reward
        """

        # start from existing weights
        if soft_start:
            # load the weights
            self.q_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[0]))
            self.q_target_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[1]))

        start_training_time = time.time()

        # train the network for a number of episodes
        for episode in range(self.train_episodes):
            observation = self.env.reset()
            done = False
            self.discounted_reward = 0.0
            episode_steps = 0
            self.loss = 0.0
            self.current_episode = episode

            # while the episode is not done
            while not done:
                if render:
                    self.env.render()

                # copy of the observation to store in the replay buffer
                # because when passing the env reference, the old observation gets overwritten
                observation_copy = copy.copy(observation)
                action = self.policy(observation, 'epsilon_greedy')
                new_observation, reward, done = self.env.step(self.env.actions[action])
                new_observation_copy = copy.copy(new_observation)
                self.discounted_reward += self.discount_factor**episode_steps * reward
                
                # store the experience in the replay buffer
                self.replay_buffer.store_tuple(observation_copy, action, reward, new_observation_copy, done)
                observation = new_observation_copy

                # train the network every train_rate_steps
                if self.total_step_counter % self.train_rate_steps == 0 or done:
                    self.train()
                
                self.total_step_counter += 1
                episode_steps += 1
            
            # update the epsilon
            self.epsilon = self.epsilon - self.epsilon_decay if self.epsilon > self.epsilon_final else self.epsilon_final
            # update the target network
            if (episode+1) % self.update_rate_episodes == 0:
                self.q_target_net.model.set_weights(self.q_net.model.get_weights()) 
            # save the weights every 10 episodes
            if episode % 10 == 0:
                self.q_net.model.save_weights(os.path.join(self.weights_folder, self.weights_name[0]))
                self.q_target_net.model.save_weights(os.path.join(self.weights_folder, self.weights_name[1]))
                # clear the session to avoid memory leaks
                K.clear_session()    
            
            # save the metrics to tensorboard
            with self.writer.as_default():
                tf.summary.scalar('loss', self.loss, step=episode)
                tf.summary.scalar('epsilon', self.epsilon, step=episode)
                tf.summary.scalar('reward', self.discounted_reward, step=episode)
                tf.summary.scalar('episode_steps', episode_steps, step=episode)
            self.writer.flush()
               
        self.training_time = time.time() - start_training_time
        print("Training time: {}".format(self.training_time))
            
    def evaluate_model(self, episodes, swingUp=False, render=True, plot=True, verbose=False, final=False):
        """
        Evaluate the model for a number of episodes
        """

        # load the weights from the final results folder
        if final:
            self.q_net.model.load_weights(os.path.join(self.final_weights_folder, self.weights_name[0]))
            self.q_target_net.model.load_weights(os.path.join(self.final_weights_folder, self.weights_name[1]))
        else:
            self.q_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[0]))
            self.q_target_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[1]))

        theta_list = []
        theta_dot_list = []
        torque_list = []

        for episode in range(episodes):

            # set the environment to the initial state (theta=0, theta_dot=0)
            if swingUp:
                # observation = self.env.reset_swingUp()
                observation = self.env.reset_robot(mode="home")
            else:
                observation = self.env.reset()
            done = False
            episode_reward = 0

            # evaluate the model for a number of steps
            while not done:
                if render:
                    self.env.render()
                
                action = self.policy(observation, 'greedy')
                new_observation, reward, done = self.env.step(self.env.actions[action])
                new_observation_copy = copy.copy(new_observation)
                episode_reward += reward
                
                observation = new_observation_copy
                
                # append the angle, angular velocity and torque to the lists
                if self.nJoint == 1:
                    theta_list.append(observation[0])
                    theta_dot_list.append(observation[1])
                    torque_list.append(self.env.actions[action])
                else:
                    theta_list.append([observation[0], observation[2]])
                    theta_dot_list.append([observation[1], observation[3]])
                    torque_list.append([self.env.actions[action], 0.0])

                if verbose:
                    print("Episode: {}, Step: {}, Reward: {}".format(episode, self.env.iterCount, episode_reward))
        
        if plot:
            # plot the angle, angular velocity and torque using sns
            sns.set()
            # plot the angle
            # if the pendulum is single
            if self.nJoint == 1:
                # plot the angles
                plt.plot(theta_list)
                plt.xlabel("Steps")
                plt.ylabel("Angle")
                plt.legend(["q"])
                plt.title("Swing Up Angle")
                plt.show()
                # plot the angular velocities
                plt.plot(theta_dot_list)
                plt.xlabel("Steps")
                plt.ylabel("Angular Velocity")
                plt.legend(["dq"])
                plt.title("Swing Up Angular Velocity")
                plt.show()
                # plot the torques
                plt.plot(torque_list)
                plt.xlabel("Steps")
                plt.ylabel("Torque")
                plt.legend(["tau"])
                plt.title("Swing Up Torque")
                plt.show()
            # if the pendulum is double
            else:
                # plot the angles
                plt.plot(theta_list)
                plt.xlabel("Steps")
                plt.ylabel("Angles")
                plt.legend(["q1", "q2"])
                plt.title("Swing Up Angles")
                plt.show()
                # plot the angular velocities
                plt.plot(theta_dot_list)
                plt.xlabel("Steps")
                plt.ylabel("Angular Velocities")
                plt.legend(["dq1", "dq2"])
                plt.title("Swing Up Angular Velocities")
                plt.show()
                # plot the torques
                plt.plot(torque_list)
                plt.xlabel("Steps")
                plt.ylabel("Torques")
                plt.legend(["tau1", "tau2"])
                plt.title("Swing Up Torques")
                plt.show()

    def plot_value_policy(self, visual='2D', resolution=10, final=False):
        """
        Plot the value function and the policy of single pendulum
        """
        # Load the weights from the final results folder
        if final:
            self.q_net.model.load_weights(os.path.join(self.final_weights_folder, self.weights_name[0]))
            self.q_target_net.model.load_weights(os.path.join(self.final_weights_folder, self.weights_name[1]))
        else:
            self.q_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[0]))
            self.q_target_net.model.load_weights(os.path.join(self.weights_folder, self.weights_name[1]))

        # Discretize the state space
        theta = np.linspace(-np.pi, np.pi, resolution)
        theta_dot = np.linspace(-self.env.vmax, self.env.vmax, resolution)

        # Create meshgrid
        theta_mesh, theta_dot_mesh = np.meshgrid(theta, theta_dot)

        # Initialize value function and policy arrays
        V = np.zeros_like(theta_mesh)
        P = np.zeros_like(theta_mesh)

        # Iterate over each state in the meshgrid
        for i in range(resolution):
            for j in range(resolution):
                state = np.array([theta_mesh[i, j], theta_dot_mesh[i, j]])
                state_tensor = tf.constant(state, dtype=tf.float32)
                q_values = self.q_net.model(state_tensor[None])[0]
                V[i, j] = tf.reduce_max(q_values)
                P[i, j] = tf.argmax(q_values)
                # map the action index to the action value
                P[i, j] = self.env.actions[int(P[i, j])]
        
        if visual=='3D':
            # Set the viewing angles
            elevation = 90  # Viewing angle from above
            azimuth = -90  # Rotate around the z-axis

            # Create 3D plots
            fig = plt.figure(figsize=(10, 5))
            ax1 = fig.add_subplot(121, projection='3d')
            value_surf = ax1.plot_surface(theta_mesh, theta_dot_mesh, V, cmap=cm.viridis)
            ax1.view_init(elevation, azimuth)  # Set the viewing angles
            ax1.set_xlabel('q')
            ax1.set_ylabel('dq')
            ax1.set_zlabel('Value')
            ax1.set_title('Value Function')
            fig.colorbar(value_surf, shrink=0.5, aspect=5)

            ax2 = fig.add_subplot(122, projection='3d')
            policy_surf = ax2.plot_surface(theta_mesh, theta_dot_mesh, P, cmap=cm.Spectral)
            ax2.view_init(elevation, azimuth)  # Set the viewing angles
            ax2.set_xlabel('q')
            ax2.set_ylabel('dq')
            ax2.set_zlabel('Action')
            ax2.set_title('Policy Function')
            fig.colorbar(policy_surf, shrink=0.5, aspect=5)
        else:
            # Set Seaborn style
            sns.set()

            # Create 2D plots with colormaps using Seaborn
            fig, axes = plt.subplots(1, 2, figsize=(10, 5))

            # Plot the value function
            ax1 = axes[0]
            sns.heatmap(V, cmap='viridis', ax=ax1, cbar=True)
            # set ticks as theta and theta_dot
            ax1.set_xticks(np.linspace(0, resolution, 5))
            ax1.set_xticklabels([-3, -1, 0, 1, 3])
            ax1.set_yticks(np.linspace(0, resolution, 5))
            ax1.set_yticklabels(np.linspace(-self.env.vmax, self.env.vmax, 5, dtype=int))
            ax1.set_xlabel('q')
            ax1.set_ylabel('dq')
            ax1.set_title('Value Function')

            # Plot the policy
            ax2 = axes[1]
            sns.heatmap(P, cmap='Spectral', ax=ax2, cbar=True)
            # set ticks as theta and theta_dot
            ax2.set_xticks(np.linspace(0, resolution, 5))
            ax2.set_xticklabels([-3, -1, 0, 1, 3])
            ax2.set_yticks(np.linspace(0, resolution, 5))
            ax2.set_yticklabels(np.linspace(-self.env.vmax, self.env.vmax, 5, dtype=int))
            ax2.set_xlabel('q')
            ax2.set_ylabel('dq')
            ax2.set_title('Policy Function')
            plt.tight_layout()

        plt.show()

    def parse_ini(self, ini_file):
        """
        Parse the ini file with the env parameters
        """
        config = configparser.ConfigParser()
        config.read(ini_file)

        if self.env.name == 'RealPendulum' or self.env.name == 'PyBulletPendulum' or self.env.name == '1-fakeenv':
            # parse the 'rotary_pendulum' section
            return config['rotary_pendulum']
        else:
            # raise an exception if the environment is not supported
            return Exception("Environment not supported")
    
    def save_metrics(self, episode, episode_reward, last_loss, last_epsilon, episode_time):
        """
        Save the metrics in a dataframe and export it to a csv file
        """
        # if the dataframe is empty, create it
        if self.metrics_df.empty:
            self.metrics_df = pd.DataFrame(columns=['episode', 'reward', 'loss', 'epsilon', 'time'])
            timestamp_ep = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.metrics_name = self.name_model + '_' + timestamp_ep + '.csv'
        
        # append the metrics to the dataframe using iloc
        self.metrics_df.loc[len(self.metrics_df)] = [episode, episode_reward, last_loss, last_epsilon, episode_time]
        # export the dataframe to a csv file with timestamp
        self.metrics_df.to_csv(os.path.join(self.metrics_folder, self.metrics_name), index=False)

        

    
        
