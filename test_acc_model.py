from stable_baselines3 import PPO
from train_acc_with_rl import CarlaACCEnvWithHUD


def test_model():
    """
    Testa un modello PPO addestrato in uno scenario specifico utilizzando l'ambiente CARLA.
    Visualizza in tempo reale lo streaming della camera e l'HUD con informazioni sulla simulazione.

    Il modello caricato viene eseguito per 1000 step o fino alla fine dell'episodio (collisione o altra condizione di termine).

    Returns:
        None
    """
    # Carica il modello PPO addestrato
    model = PPO.load("ppo_acc_model_highway")

    # Inizializza l'ambiente di test con HUD (scenario urbano di default)
    env = CarlaACCEnvWithHUD(scenario_type="city")

    # Resetta l'ambiente e ottieni lo stato iniziale
    obs = env.reset()

    # Ciclo di test: esegue il modello per 1000 step
    for _ in range(1000):
        # Predici l'azione basandoti sullo stato attuale
        action, _ = model.predict(obs)

        # Esegui l'azione nell'ambiente
        obs, rewards, done, info = env.step(action)

        # Se l'episodio è terminato (es. collisione), resetta l'ambiente
        if done:
            obs = env.reset()

    # Chiudi l'ambiente al termine del test
    env.close()


if __name__ == "__main__":
    """
    Entrypoint dello script per eseguire il test del modello PPO addestrato.
    Assicurati che il modello "ppo_acc_model_highway" esista prima di eseguire il test.

    Steps:
    1. Carica il modello addestrato.
    2. Esegui il test con HUD e streaming video in tempo reale.
    3. Mostra le informazioni HUD (velocità, distanza relativa, reward) durante il test.
    """
    test_model()
