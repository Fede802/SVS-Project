import gym
import numpy as np
import carla
import pygame
import random
import math
from stable_baselines3 import PPO
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import CheckpointCallback


# Funzione per calcolare la velocità del veicolo
def get_speed(vehicle):
    """
    Calcola la velocità attuale del veicolo basandosi sui componenti vettoriali.

    Args:
        vehicle (carla.Vehicle): Il veicolo di cui calcolare la velocità.

    Returns:
        float: La velocità del veicolo in m/s.
    """
    velocity = vehicle.get_velocity()
    return math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)


# Funzione per calcolare la velocità relativa
def calculate_relative_speed(ego_velocity, target_velocity):
    """
    Calcola la velocità relativa tra il veicolo controllato e un target.

    Args:
        ego_velocity (float): Velocità del veicolo controllato (ego).
        target_velocity (float): Velocità del veicolo target.

    Returns:
        float: La velocità relativa.
    """
    return ego_velocity - target_velocity


# Funzione per calcolare la distanza relativa dal radar
def get_relative_distance(radar_data):
    """
    Calcola la distanza relativa più vicina dai dati radar.

    Args:
        radar_data (list): Dati radar contenenti le profondità rilevate.

    Returns:
        float: La distanza relativa più vicina in metri.
    """
    distances = [d.depth for d in radar_data]
    if distances:
        return min(distances)
    return None  # Nessun veicolo rilevato


# Classe HUD per visualizzare informazioni
class HUD:
    """
    Classe per la gestione di un Head-Up Display (HUD) utilizzando pygame.
    Visualizza informazioni come velocità, distanza e reward direttamente nella finestra.

    Attributes:
        width (int): Larghezza della finestra pygame.
        height (int): Altezza della finestra pygame.
        display (pygame.Surface): Superficie pygame per il rendering.
        font (pygame.font.Font): Font per il testo.
        info_text (str): Testo informativo da visualizzare.
    """

    def __init__(self, width, height):
        """
        Inizializza il sistema HUD.

        Args:
            width (int): Larghezza della finestra pygame.
            height (int): Altezza della finestra pygame.
        """
        pygame.init()
        self.width = width
        self.height = height
        self.display = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("CARLA RL - HUD Display")
        self.font = pygame.font.Font(None, 24)
        self.info_text = ""

    def render(self, camera_image, info_text):
        """
        Esegue il rendering dello streaming della camera e del testo informativo sull'HUD.

        Args:
            camera_image (carla.Image): Immagine RGB catturata dalla camera.
            info_text (str): Testo informativo da visualizzare (es. velocità, reward).
        """
        # Converti immagine CARLA (BGRA) in un formato compatibile con pygame (RGB)
        array = np.frombuffer(camera_image.raw_data, dtype=np.uint8)
        array = array.reshape((camera_image.height, camera_image.width, 4))
        array = array[:, :, :3][:, :, ::-1]

        # Disegna immagine
        surface = pygame.surfarray.make_surface(array.swapaxes(0, 1))
        self.display.blit(surface, (0, 0))

        # Disegna testo
        self.info_text = info_text
        for i, line in enumerate(self.info_text.splitlines()):
            text_surface = self.font.render(line, True, (255, 255, 255))
            self.display.blit(text_surface, (10, 10 + i * 20))

        pygame.display.flip()


# Ambiente personalizzato CARLA con HUD
class CarlaACCEnvWithHUD(gym.Env):
    """
    Ambiente personalizzato per l'addestramento di un sistema Adaptive Cruise Control (ACC)
    in CARLA. Integra un HUD e uno streaming della camera RGB.

    Attributes:
        scenario_type (str): Tipo di scenario (es. highway, city, rural).
        hud (HUD): Istanza del sistema HUD.
    """

    def __init__(self, scenario_type="highway"):
        """
        Inizializza l'ambiente CARLA con il veicolo controllato, il radar, la camera e il traffico.

        Args:
            scenario_type (str): Tipo di scenario da simulare (highway, city, rural).
        """
        super(CarlaACCEnvWithHUD, self).__init__()

        # Connetti a CARLA
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)
        self.world = self.client.get_world()

        # Blueprint del veicolo
        blueprint_library = self.world.get_blueprint_library()
        self.vehicle_bp = blueprint_library.filter('vehicle.tesla.model3')[0]

        # Spawn veicolo
        spawn_points = self.world.get_map().get_spawn_points()
        self.vehicle = self.world.spawn_actor(self.vehicle_bp, random.choice(spawn_points))

        # Radar sensor
        radar_bp = blueprint_library.find('sensor.other.radar')
        radar_transform = carla.Transform(carla.Location(x=2.5, z=1.0))
        self.radar = self.world.spawn_actor(radar_bp, radar_transform, attach_to=self.vehicle)
        self.radar.listen(self.radar_callback)

        # Camera sensor
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute("image_size_x", str(800))
        camera_bp.set_attribute("image_size_y", str(600))
        self.camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        self.camera = self.world.spawn_actor(camera_bp, self.camera_transform, attach_to=self.vehicle)
        self.camera.listen(self.camera_callback)

        # HUD
        self.hud = HUD(800, 600)

        # Setup traffico dinamico
        self.setup_traffic(scenario_type)

        # Dati radar e camera
        self.radar_data = None
        self.camera_image = None

        # Azioni (accelerazione e frenata)
        self.action_space = gym.spaces.Box(low=np.array([0, 0]), high=np.array([1, 1]), dtype=np.float32)

        # Stato (velocità attuale, distanza relativa, velocità relativa)
        self.observation_space = gym.spaces.Box(low=0, high=1, shape=(3,), dtype=np.float32)

        # Parametri per normalizzazione
        self.max_speed = 30.0  # 30 m/s (circa 108 km/h)
        self.max_distance = 100.0  # Distanza massima in metri
        self.max_relative_speed = 30.0  # Velocità relativa massima

    def setup_traffic(self, scenario_type):
        """
        Configura il traffico dinamico nello scenario.

        Args:
            scenario_type (str): Tipo di scenario (highway, city, rural).
        """
        blueprint_library = self.world.get_blueprint_library()

        if scenario_type == "highway":
            for i in range(10):
                spawn_point = random.choice(self.world.get_map().get_spawn_points())
                npc_vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
                npc_vehicle = self.world.try_spawn_actor(npc_vehicle_bp, spawn_point)
                if npc_vehicle:
                    npc_vehicle.set_autopilot(True)

        elif scenario_type == "city":
            for i in range(50):
                spawn_point = random.choice(self.world.get_map().get_spawn_points())
                npc_vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
                npc_vehicle = self.world.try_spawn_actor(npc_vehicle_bp, spawn_point)
                if npc_vehicle:
                    npc_vehicle.set_autopilot(True)

        elif scenario_type == "rural":
            weather = carla.WeatherParameters(
                cloudiness=80.0,
                precipitation=60.0,
                fog_density=40.0
            )
            self.world.set_weather(weather)
            for i in range(15):
                spawn_point = random.choice(self.world.get_map().get_spawn_points())
                npc_vehicle_bp = random.choice(blueprint_library.filter('vehicle'))
                npc_vehicle = self.world.try_spawn_actor(npc_vehicle_bp, spawn_point)
                if npc_vehicle:
                    npc_vehicle.set_autopilot(True)

    def radar_callback(self, data):
        """Callback per aggiornare i dati radar."""
        self.radar_data = data

    def camera_callback(self, image):
        """Callback per aggiornare i dati della camera RGB."""
        self.camera_image = image

    def reset(self):
        """
        Reset dello scenario e stato iniziale.

        Returns:
            np.array: Stato iniziale normalizzato.
        """
        spawn_points = self.world.get_map().get_spawn_points()
        self.vehicle.set_transform(random.choice(spawn_points))
        self.vehicle.set_autopilot(False)

        self.radar_data = None
        self.camera_image = None

        return self.get_observation()

    def get_observation(self):
        """
        Restituisce lo stato del veicolo, inclusi velocità attuale, distanza relativa e velocità relativa.

        Returns:
            np.array: Stato normalizzato.
        """
        ego_speed = get_speed(self.vehicle)

        if self.radar_data:
            relative_distance = get_relative_distance(self.radar_data)
            relative_speed = calculate_relative_speed(ego_speed, 0.0)
        else:
            relative_distance = self.max_distance
            relative_speed = 0.0

        current_speed = min(ego_speed, self.max_speed) / self.max_speed
        relative_distance = min(relative_distance, self.max_distance) / self.max_distance
        relative_speed = (relative_speed + self.max_relative_speed) / (2 * self.max_relative_speed)

        return np.array([current_speed, relative_distance, relative_speed], dtype=np.float32)

    def step(self, action):
        """
        Applica l'azione al veicolo, aggiorna lo stato e calcola la reward.

        Args:
            action (np.array): Azione composta da throttle e brake.

        Returns:
            tuple: Stato successivo, reward, done flag e informazioni aggiuntive.
        """
        throttle, brake = action
        control = carla.VehicleControl(throttle=float(throttle), brake=float(brake))
        self.vehicle.apply_control(control)

        obs = self.get_observation()
        reward = self.compute_reward(obs, action)
        done = self.check_done(obs)

        # Render HUD
        if self.camera_image:
            speed = get_speed(self.vehicle)
            hud_text = f"Speed: {speed:.2f} m/s\nReward: {reward:.2f}"
            if self.radar_data:
                hud_text += f"\nDistance: {get_relative_distance(self.radar_data):.2f} m"
            self.hud.render(self.camera_image, hud_text)

        return obs, reward, done, {}

    def compute_reward(self, obs, action):
        """
        Calcola la reward basandosi sulla distanza relativa, velocità e azioni.

        Args:
            obs (np.array): Stato attuale.
            action (np.array): Azione eseguita.

        Returns:
            float: Valore della reward.
        """
        current_speed, relative_distance, relative_speed = obs

        distance_penalty = -10 if relative_distance < 0.1 else 0
        brake_penalty = -action[1] if relative_distance > 0.5 else 0
        throttle_penalty = -action[0] if relative_distance < 0.3 else 0

        safety_reward = 1 - abs(relative_distance - 0.5)
        speed_reward = 1 - abs(current_speed - 0.5)

        return safety_reward + speed_reward + distance_penalty + brake_penalty + throttle_penalty

    def check_done(self, obs):
        """
        Controlla se l'episodio deve terminare (collisione o distanza troppo bassa).

        Args:
            obs (np.array): Stato attuale.

        Returns:
            bool: True se l'episodio deve terminare, False altrimenti.
        """
        _, relative_distance, _ = obs
        return relative_distance < 0.05

    def close(self):
        """Chiude l'ambiente e distrugge gli attori CARLA."""
        self.radar.destroy()
        self.vehicle.destroy()
        pygame.quit()


# Funzione per addestrare il modello
def train_model():
    """
    Addestra un modello PPO utilizzando l'ambiente CarlaACCEnvWithHUD.
    Salva il modello e i checkpoint periodici.
    """
    env = DummyVecEnv([lambda: CarlaACCEnvWithHUD(scenario_type="highway")])
    checkpoint_callback = CheckpointCallback(save_freq=1000, save_path='./checkpoints/', name_prefix='ppo_acc')
    model = PPO("MlpPolicy", env, verbose=1)
    model.learn(total_timesteps=50000, callback=checkpoint_callback)
    model.save("ppo_acc_model_highway")
    env.close()


# Funzione per testare il modello
def test_model():
    """
    Testa il modello addestrato in uno scenario specifico.
    Visualizza i risultati tramite HUD e camera streaming.
    """
    model = PPO.load("ppo_acc_model_highway")
    env = CarlaACCEnvWithHUD(scenario_type="city")
    obs = env.reset()
    for _ in range(1000):
        action, _ = model.predict(obs)
        obs, rewards, done, info = env.step(action)
        if done:
            obs = env.reset()
    env.close()


# Entrypoint dello script
if __name__ == "__main__":
    choice = input("Scegli: train (t) o test (s): ").strip()
    if choice == 't':
        train_model()
    elif choice == 's':
        test_model()
