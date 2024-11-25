import numpy as np
import gymnasium as gym
from gymnasium import spaces
import carla

#Template for enviroment
class CarlaEnv(gym.Env):
    
    def __init__(self):
        super(CarlaEnv, self).__init__()
        
        #Initialize carla client and world
        self.client =  carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()
        
        #Configure space actions and observation
        self.action_space = spaces.Box(low=np.array([-1.0, 0.0]), high=np.array([1.0, 1.0]), dtype=np.float32)  # [-1,1] steering e [0,1] throttle
        self.observation_space = spaces.Box(low=0, high=255, shape=(84, 84, 3), dtype=np.uint8)  # Es: immagine 84x84 RGB
        
        # Configura il veicolo
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = self.blueprint_library.filter('vehicle.tesla.model3')[0]
        self.spawn_points = self.world.get_map().get_spawn_points()
        self.vehicle = None
    
    def reset(self):
        # Ripristina l'ambiente
        if self.vehicle is not None:
            self.vehicle.destroy()

        spawn_point = np.random.choice(self.spawn_points)
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, spawn_point)

        # Restituisci lo stato iniziale (esempio: immagine dalla telecamera)
        return self._get_observation(), {}
    
    def step(self, action):
        # Esegui azioni sull'auto
        throttle = float(action[0])
        steer = float(action[1])
        control = carla.VehicleControl(throttle=throttle, steer=steer)
        self.vehicle.apply_control(control)

        # Aggiorna osservazioni e calcola reward
        observation = self._get_observation()
        reward = self._compute_reward()
        done = self._check_done()

        return observation, reward, done, False, {}
    
    def _get_observation(self):
        # Esempio: ottieni immagine dalla telecamera
        return np.zeros((84, 84, 3), dtype=np.uint8)  # Sostituisci con immagine reale

    def _compute_reward(self):
        # Definisci una funzione di reward
        return 1.0  # Sostituisci con logica basata sugli obiettivi

    def _check_done(self):
        # Definisci la logica per terminare l'episodio
        return False  # Cambia condizione

    def close(self):
        if self.vehicle is not None:
            self.vehicle.destroy()