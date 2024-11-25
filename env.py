import gymnasium as gym

#Template for enviroment
class CarEnv(gym.env):
    
    """CLASS COSTANTS"""
    
    def __init__(self):
        super(CarEnv, self).__init__()
    
    #Define actions and observation space
    #self.action_space = ...
    
    #Define discrete variables with 4 possible actions for throttle/breaking
    
    #Define observation:
    #self.observation_space = ...
    
    #Connect to simulator to spawn car
    #self.client = carla.Client("localhost", 2000)
    #self.client.set_timeout(4.0)
    
    #Define step
        #Process latest from this step
        #Define reward
        #If found episode end (collision), done = True and cleanup
        #return observation(camera), reward, done, {}
    
    #Define reset(self):
        # spawn a car, attach sensor etc
        # return observation(camera)