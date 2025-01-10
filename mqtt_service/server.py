import json, threading, http.server, socketserver
import paho.mqtt.client as mqtt

# Configurazione MQTT
BROKER = "5cdd7fb827074230b4088a9be79ac978.s1.eu.hivemq.cloud"
# BROKER = "test.mosquitto.org" #"localhost" # Broker MQTT (usare localhost se è locale)
PORT = 8883           # Porta del broker MQTT
TOPIC = "svsProject/status"  # Topic MQTT su cui pubblicare
client = mqtt.Client()
client.username_pw_set("admin", "Admin111")

class CustomHttpRequestHandler(http.server.SimpleHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.path = 'index.html'
        return http.server.SimpleHTTPRequestHandler.do_GET(self)
    
# Funzione per avviare il server HTTP
def start_http_server():
    handler = CustomHttpRequestHandler
    with socketserver.TCPServer(("0.0.0.0", 8001), handler) as httpd:
        print("Server HTTP avviato sulla porta 8000")
        httpd.serve_forever()
    
def start_servers():
    # Connessione al broker
    client.connect(BROKER, PORT, 60)
    print("Connessione riuscita: ", client.is_connected())
    # Avvia il server HTTP in un thread separato
    # http_thread = threading.Thread(target=start_http_server)
    # http_thread.daemon = True  # Il thread si chiuderà quando il programma termina
    # http_thread.start()

def send_data(data, topic=TOPIC):
    # Converte i dati in formato JSON e li pubblica
   # print("Sending data: ", data)
    client.publish(topic, json.dumps(data))

def close_servers():
    client.disconnect()

