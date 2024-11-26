from stable_baselines3 import PPO
from env import CarlaEnv

# Load the environment and the trained model
env = CarlaEnv()
model = PPO.load("ppo_carla_model")

# Run a few episodes with the trained model
for episode in range(5):
    observation = env.reset()
    done = False
    total_reward = 0

    while not done:
        # Use the trained model to predict actions
        action, _states = model.predict(observation, deterministic=True)
        observation, reward, done, info = env.step(action)
        print(f"Observation shape: {observation.shape}")
        total_reward += reward

    print(f"Episode {episode + 1} Total Reward: {total_reward}")

# Clean up the environment
env.close()
