from env import CarlaEnv

env = CarlaEnv()

obs, info = env.reset()
done = False

while not done:
    action = env.action_space.sample()  # Azione casuale
    obs, reward, done, truncated, info = env.step(action)
    print(f"Reward: {reward}")

env.close()
