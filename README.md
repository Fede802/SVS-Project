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

  https://prod.liveshare.vsengsaas.visualstudio.com/join?890F73ED1FE9E42300B00D236264FF532C1A