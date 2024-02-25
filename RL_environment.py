import gymnasium as gym
from gymnasium import spaces
import numpy as np

from gymnasium.envs.registration import register

from vehicle_dynamics import Vehicle



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

    def step(self, action):
        # Execute one time step within the environment
        self.vehicle.update_dynamics(dt=0.1, input_command={'throttle': action[1], 'steering': action[0]})
        state = self.vehicle.get_state()

        # You should adjust the observation according to your state representation
        observation = np.array([state['x'], state['y'], state['yaw'], state['v_x'], state['v_y'], state['r'], state['a_x'], state['a_y'], state['steer']])

        # Calculate reward (you'll need to implement this based on your criteria)
        reward = self.calculate_reward(state)
        done = self.is_done(state)
        terminated = self.is_done(state)
        truncated = False  # Placeholder for now

        return observation, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        # Reset the state of the environment to an initial state
        # TODO: implement this in your Vehicle class
        self.vehicle.reset(seed, options)
        state = self.vehicle.get_state()
        observation = np.array([state['x'], state['y'], state['yaw'], state['v_x'], state['v_y'], state['r'], state['a_x'], state['a_y'], state['steer']])
        return observation, {}

    def render(self, mode='console'):
        # Render the environment (optional)
        # For simplicity, let's just print some information about the state
        if mode != 'console':
            raise NotImplementedError("Only 'console' mode is supported for rendering")
        state = self.vehicle.get_state()
        print(f"Vehicle position: ({state['x']}, {state['y']})")

    def calculate_reward(self, state):
        # Implement reward calculation
        # Placeholder implementation
        return 0

    def is_done(self, state):
        # Implement logic to determine the terminal state
        # Placeholder implementation
        return False

    def close(self):
        # Clean up resources, if necessary
        self.vehicle.Destructor()

register(
    id='AMZCarControls-v0',
    entry_point='RL_environment:CustomAMZEnv',
)

