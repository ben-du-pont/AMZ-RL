import gymnasium as gym


import os
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.evaluation import evaluate_policy
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.monitor import Monitor
# Import your custom environment
# Make sure your custom environment is registered in Gym before this step
# For example, the environment registration should be done in the file where your environment class is defined
import RL_environment

# Setup directory to save trained models
model_dir = "trained_models"
os.makedirs(model_dir, exist_ok=True)

# Create the environment
env_id = 'AMZCarControls-v0'
env = make_vec_env(env_id, n_envs=100)  # Use vectorized environments to speed up training

# Initialize the agent
model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_car_tensorboard/")

print("Model initialized")

# Train the agent
total_timesteps = 1e6  # Adjust based on the complexity of your environment and the hardware capabilities
model.learn(total_timesteps=int(total_timesteps))
print("Model learned")
# Save the trained model
model_path = os.path.join(model_dir, "ppo_car_control")
model.save(model_path)

# Optionally, evaluate the trained model
eval_env = gym.make(env_id)
eval_env = Monitor(eval_env)


# Render the environment during testing
obs, _ = eval_env.reset()
for _ in range(1000):  # You can adjust the number of steps to render
    
    action, _ = model.predict(obs, deterministic=True)
    obs, reward, done, truncated, info = eval_env.step(action)


mean_reward, std_reward = evaluate_policy(model, eval_env, n_eval_episodes=10)
print(f"Mean reward: {mean_reward}, Std reward: {std_reward}")

# Close the environment
env.close()
