import time
import random
import json
import paho.mqtt.client as mqtt
import threading
import http.server
import socketserver

# Configurazione MQTT
BROKER = "localhost"
# BROKER = "test.mosquitto.org" #"localhost" # Broker MQTT (usare localhost se è locale)
PORT = 1883           # Porta del broker MQTT
TOPIC = "grafico/dati/sdahfkjhdkjfhals"  # Topic MQTT su cui pubblicare
client = mqtt.Client()

# Funzione per avviare il server HTTP
def start_http_server():
    handler = http.server.SimpleHTTPRequestHandler
    with socketserver.TCPServer(("0.0.0.0", 8000), handler) as httpd:
        print("Server HTTP avviato sulla porta 8000")
        httpd.serve_forever()
    
def start_servers():
    # Connessione al broker
    client.connect(BROKER, PORT, 60)
    # Avvia il server HTTP in un thread separato
    http_thread = threading.Thread(target=start_http_server)
    http_thread.daemon = True  # Il thread si chiuderà quando il programma termina
    http_thread.start()

def send_data(data):
    # Converte i dati in formato JSON e li pubblica
    client.publish(TOPIC, json.dumps(data))

def close_servers():
    client.disconnect()

