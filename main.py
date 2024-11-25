import time
from env import CarlaEnv 

def test_carla_env():
    env = CarlaEnv()
    # Reset the environment
    observation = env.reset()
    print("Initial Observation:", observation)
        
    for _ in range(1000000):  # Run the environment for 10 steps
        action = env.action_space.sample()  # Take a random action
        print(f"Taking action: {action}")
            
        # Step in the environment
        observation, reward, done, info = env.step(action)
        print(f"Observation: {observation}, Reward: {reward}, Done: {done}")
            
        if done:
            print("Episode ended. Resetting environment.")
            observation = env.reset()
                
        time.sleep(0.1)  # Wait a bit to visualize the simulation

if __name__ == "__main__":
    test_carla_env()
