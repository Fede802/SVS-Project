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

## Invariants
- If the vehicle come to halt completely, the system can automatically start the vehicle forward within 3 seconds, if traffic allows.
- If vehicle is in halt more than 3 seconds (the time can vary as per manufacturer), then driver has to either press the button to reactivate the ACC with stop&go or press gas acceleration paddle to start the vehicle.

Acc Schema Image:
  <a href="">
    <img src="imgs/acc-schema.png" alt="Schema" width="auto" height="256" />
  </a>

env step call
reward piccoli



## TODO
- [ ] Improvement controlli (retro, setup cruise)
- [ ] Testare i pedali e resize in lab
- [ ] Valutare world sync vs async
- [ ] Migliorare traffico che si ribaltano
- [ ] Miglioramento mqtt
- [ ] Multi-radar
- [ ] Opt: braking model con 1/ttc invece della distanza
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

