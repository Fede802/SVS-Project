import json, threading, http.server, socketserver
import paho.mqtt.client as mqtt

# Configurazione MQTT
BROKER = "broker.hivemq.com"
# BROKER = "test.mosquitto.org" #"localhost" # Broker MQTT (usare localhost se è locale)
PORT = 1883           # Porta del broker MQTT
TOPIC = "svsProject/status"  # Topic MQTT su cui pubblicare
client = mqtt.Client()
# Uncomment for ad-hoc connection
# client.username = "admin"
# client.password = "Admin111"
# client.tls_set()

class CustomHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = 'mqtt_service/index.html'
        return http.server.SimpleHTTPRequestHandler.do_GET(self)
    
# Funzione per avviare il server HTTP
def start_http_server():
    handler = CustomHttpRequestHandler
    with socketserver.TCPServer(("0.0.0.0", 8080), handler) as httpd:
        print("Server HTTP avviato sulla porta 8080")
        httpd.serve_forever()
    
def start_servers():
    # Connessione al broker
    client.on_connect_failure = lambda client, userdata, flags, rc: print("Connessione fallita: ", rc)
    client.on_connect = lambda client, userdata, flags, rc: print("Connesso con codice: ", rc)
    client.connect(BROKER, PORT, 60)
    
    # Avvia il server HTTP in un thread separato
    http_thread = threading.Thread(target=start_http_server)
    http_thread.daemon = True  # Il thread si chiuderà quando il programma termina
    http_thread.start()

def send_data(data, topic=TOPIC):
    # Converte i dati in formato JSON e li pubblica
    # print("Sending data: ", data)
    client.publish(topic, json.dumps(data))

def close_servers():
    client.disconnect()

