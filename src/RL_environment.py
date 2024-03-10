import gymnasium as gym
from gymnasium import spaces
import numpy as np

from gymnasium.envs.registration import register

from vehicle_dynamics import Vehicle
from path import Path
from render import Renderer
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter


class CustomAMZEnv(gym.Env):
    metadata = {'render_modes': ['console']}

    def __init__(self):
        super(CustomAMZEnv, self).__init__()

        # Define action and observation space

        # Actions: continuous steering angle [-0.45, 0.45] and acceleration [-5.0, 10.0]
        self.action_space = spaces.Box(low=np.array([-0.45, -5.0]), high=np.array([0.45, 10.0]), dtype=np.float32)

        # Define an example observation space comprising position, velocity, acceleration, etc.

        # You should adjust this according to the actual state variables of your Vehicle
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(9,), dtype=np.float64)

        # Initialize the Vehicle dynamics simulator
        self.vehicle = Vehicle()
        self.path = Path()
        self.renderer = Renderer(self.path, self.vehicle)
        self.step_number = 0
        self.anim = None
        self.car_outside_since = 0

    def step(self, action):
        # Execute one time step within the environment
        self.vehicle.update_dynamics(dt=0.1, input_command={'throttle': action[1], 'steering': action[0]})
        state = self.vehicle.get_state()
        distance_left, distance_right = self.path.calculate_distance_to_boundaries(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])
        track_width, _ = self.path.get_track_width(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])

        if distance_left > track_width + 0.1 or distance_right > track_width + 0.1:
            self.car_outside_since += 1
        else:
            self.car_outside_since = 0

        if self.car_outside_since > 20:
            self.car_outside_since = 0
            print("Car outside track for too long, resetting environment")
            self.reset()
            state = self.vehicle.get_state()
            distance_left, distance_right = self.path.calculate_distance_to_boundaries(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])
            track_width, _ = self.path.get_track_width(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])

            observation = np.array([state['v_x'], state['v_y'], state['r'], state['a_x'], state['a_y'], state['steer'], distance_left, distance_right, track_width])

            reward = -1e3
            terminated = self.is_done(state)
            truncated = True  # Placeholder for now
            self.step_number += 1
            return observation, reward, terminated, truncated, {}
    


        # You should adjust the observation according to your state representation
        observation = np.array([state['v_x'], state['v_y'], state['r'], state['a_x'], state['a_y'], state['steer'], distance_left, distance_right, track_width])

        # Calculate reward (you'll need to implement this based on your criteria)
        reward = self.calculate_reward(state)
        done = self.is_done(state)
        terminated = self.is_done(state)
        truncated = False  # Placeholder for now
        self.step_number += 1
        return observation, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        # Reset the state of the environment to an initial state
        # TODO: implement this in your Vehicle class
        self.vehicle.reset(seed, options)
        state = self.vehicle.get_state()
        distance_left, distance_right = self.path.calculate_distance_to_boundaries(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])
        track_width, _ = self.path.get_track_width(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])

        observation = np.array([state['v_x'], state['v_y'], state['r'], state['a_x'], state['a_y'], state['steer'], float(distance_left), float(distance_right), float(track_width)])
        self.step_number = 0
        return observation, {}

    def render(self, mode='human'):
        # Implement the render function using Matplotlib

        # Example: Matplotlib animation
        fig, ax = plt.subplots()
        line, = ax.plot([], [], lw=2)

        def init():
            line.set_data([], [])
            return line,

        def animate(i):
            # Update the plot with the current state of the environment
            # Example: Assuming your environment has a method to get the current state
            #state = self.get_current_state()
            x, y = np.random.randint(0,10),np.random.randint(0,10)  # Assuming state is a tuple of x and y coordinates
            line.set_data([x], [y])
            return line,

        anim = FuncAnimation(fig, animate, init_func=init, frames=200, interval=20, blit=True)
        plt.show()

    def calculate_reward(self, state):
        # Implement reward calculation
        distance_left, distance_right = self.path.calculate_distance_to_boundaries(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])
        track_width, _ = self.path.get_track_width(state['x'], state['y'], self.path.track_params[1], self.path.track_params[2])

        reward = 0

        if state['v_x'] > 2:
            reward += 2
        elif state['v_x'] > 4:
            reward += 4
        elif state['v_x'] > 6:
            reward += 6
        else:
            reward += 10
        
        if distance_left - track_width/2 < 0.1:
            reward -= 1
        elif distance_left - track_width/2 < 0.5:
            reward -= 2
        elif distance_left - track_width/2 < 1:
            reward -= 3
        
        if distance_right - track_width/2 < 0.1:
            reward -= 1
        elif distance_right- track_width/2 < 0.5:
            reward -= 2
        elif distance_right - track_width/2 < 1:
            reward -= 3

        if distance_left > track_width/2:
            reward -= 1e3

        if distance_right > track_width/2:
            reward -= 1e3

        return reward

    def is_done(self, state):
        # Implement logic to determine the terminal state
        # Placeholder implementation
        return self.path.lap_count > 10

    def close(self):
        # Clean up resources, if necessary
        self.vehicle.Destructor()

register(
    id='AMZCarControls-v0',
    entry_point='RL_environment:CustomAMZEnv',
)

