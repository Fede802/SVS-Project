# SVS Project: ACC with Stop & Go

## Setup

Conda environment:
```bash
conda activate carla-env
```
### Nella dir di carla
```bash
pip install -r PythonAPI/examples/requirements.txt
pip install carla
```
### Nella dir del progetto 
```bash
pip install -r requirements.txt
```
### In vscode

Cercare "conda" e mettere nel path "C:\anaconda3\Scripts\conda.exe"

## FINAL CONTROLLERS
- vehicle_controller\pid_controller\pid_controller_scheduled_following_brake_ttc.py
- vehicle_controller\pid_controller\pid_controller_random_following_brake_ttc.py
- vehicle_controller\rl_controller\rl_controller.py

## LOG
1. pid_controller_scheduled_following_brake_ttc: ego_target 110, solo caso 1, buffer 42
2. pid_controller_random_following_brake_ttc: ego_target 110, solo caso 1, buffer 42
3. pid_controller_scheduled_following_brake_ttc: ego_target 110, solo caso 1, buffer 1
4. pid_controller_scheduled_following_brake_ttc: ego_target 110, solo caso 1, buffer None 
5. pid_controller_scheduled_following_brake_ttc: ego_target 90, other_target 70, other tra 800 e 1000 frena a 1
6. rl_controller_working_braking_model: ego_target 90, other_target 70, other tra 800 e 1000 frena a 1
7. pid_controller_scheduled_following_brake_ttc: ego_target 90, other_target 70, other tra 800 e 1000 frena a 1, ranges following 50 e 10, ranges braking 20 e 0
8. rl_controller_working_braking_model: ego_target 110, solo caso 1
9. rl_controller_working_braking_model: ego_target 90, other_target 70, other tra 800 e 1000 frena a 1, ranges braking 20 e 0
10. rl_controller_working_braking_model: ego_target 90, other_target 70, other tra 800 e 1000 frena a 1, ranges braking 50 e 10

## Grafici
- ComparisonLog1e2: paragone pid scheduled e random
- ComparisonLog1e3e4: paragone diverse buffer size
- Comparison Log1e8: paragone pid scheduled e rl
- ComparisonLog5e7: scenario complesso paragone pid scheduled e pid scheduled switching
- ComparisonLog6e10: scenario complesso paragone rl e rl switching
- ComparisonLog5e6: scenario complesso paragone pid scheduled e rl
- ComparisonLog7e10: scenario complesso paragone pid scheduled switching e rl switching

## TODO
- [ ] Multi-radar
- [X] Check radar range
- [X] Fix web plot
- [X] MQTT: interfaccia e fix vari
- [X] Scenario 1: aggiungere CC e scritte per target vehicle
- [X] Valutare world sync vs async
- [X] Testare i pedali e resize in lab
- [X] Migliorare traffico che si ribaltano
- [X] Opt: braking model con 1/ttc invece della distanza
- [X] Improvement controlli (retro, setup cruise)
- [X] Miglioramento mqtt
- [X] Opt: Selezione veicoli da spawnare
- [X] Opt: Switch scenari con traffico di carla quando si preme 4 e con 3 si riavvia
- [X] Opt: check world default settings and if not try to put in sync mode
- [X] Setup TrafficManager (a che target spawnare i veicoli)
- [X] Importare Fading text per collisioni
- [X] Mostrare con cosa va a sbattere importando funzione di utility
- [X] Dentro il controlInfo, sostituire il controllo dei veicoli con i veicoli stessi (VehicleWithRadar)
- [X] Aggiungere tasto per settare CC a velocità corrente
- [X] Opt: Collision Sensor con scritte dinamiche
- [X] Traffico superstrada (ora c'è traffico con autopilot in 2 versioni)
- [X] Traffico di carla (test)
- [x] Sterzo a other vehicle
- [x] Aggiustare spawn scenario base
- [x] Movimento dinamico dello spettare di carla (avvicinamento al veicolo)
- [x] Aggiustare camera di carla (più dritto)

