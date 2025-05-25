import machine
import time
import network
from umqtt.simple import MQTTClient
from machine import Pin, PWM

# --- Configurações de Wi-Fi e MQTT ---
WIFI_SSID = 'Wokwi-GUEST'
WIFI_PASSWORD = ''  # Senha vazia para Wokwi-GUEST
MQTT_BROKER = 'broker.hivemq.com'
MQTT_PORT = 1883
MQTT_CLIENT_ID = 'esp32_sensor_cliente'
MQTT_TOPIC_PUB = b'esp32/distancia'
MQTT_TOPIC_SUB = b'esp32/comando'

# === IR Sensor (mantido para completude) ===
class IRSensor:
    def __init__(self, pin_num):
        self.sensor = Pin(pin_num, Pin.IN)

    def is_obstructed(self):
        return self.sensor.value() == 0

# === Ultrasonic Sensor HC-SR04 ===
class UltrasonicSensor:
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)

    def read_distance(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)

        timeout_us = 30000

        pulse_start = time.ticks_us()
        while self.echo.value() == 0:
            if time.ticks_diff(time.ticks_us(), pulse_start) > timeout_us:
                return -1

        pulse_end = time.ticks_us()
        while self.echo.value() == 1:
            if time.ticks_diff(time.ticks_us(), pulse_end) > timeout_us:
                return -1

        duration = time.ticks_diff(pulse_end, pulse_start)
        distance = duration / 58.2
        return distance

# === Servo Dispenser ===
class ServoDispenser:
    def __init__(self, pin_num):
        self.servo = PWM(Pin(pin_num), freq=50)
        self.servo.duty(40)

    def dispense(self):
        print("Servo: erogazione (dispensando)")
        self.servo.duty(115)
        time.sleep(0.8)
        self.servo.duty(40)
        print("Servo: chiuso (dispensado)")
        time.sleep(1)

# === Buzzer ===
class Buzzer:
    def __init__(self, pin_num):
        self.buzzer_pin = Pin(pin_num, Pin.OUT)
        self.buzzer_pin.value(0)

    def buzz(self, duration_ms):
        print(f"Buzzer: Ligando por {duration_ms} ms")
        self.buzzer_pin.value(1)
        time.sleep_ms(duration_ms)
        self.buzzer_pin.value(0)
        print("Buzzer: Desligado")

# --- Funções de Conectividade ---
client = None

def connect_wifi():
    global client
    sta_if = network.WLAN(network.STA_IF)

    if not sta_if.isconnected():
        print(f'Conectando ao Wi-Fi "{WIFI_SSID}"...')
        sta_if.active(True)
        sta_if.connect(WIFI_SSID, WIFI_PASSWORD)
        timeout = 15
        while not sta_if.isconnected() and timeout > 0:
            print('.', end='')
            time.sleep(1)
            timeout -= 1

        if sta_if.isconnected():
            print('\nConectado ao Wi-Fi!')
            print('Endereço IP:', sta_if.ifconfig()[0])
            try:
                client = MQTTClient(MQTT_CLIENT_ID, MQTT_BROKER, port=MQTT_PORT)
                client.set_callback(mqtt_callback)
                print("Tentando conectar ao broker MQTT...")
                client.connect()
                client.subscribe(MQTT_TOPIC_SUB)
                print(f'Conectado ao broker MQTT {MQTT_BROKER} e inscrito no tópico {MQTT_TOPIC_SUB.decode()}')
            except OSError as e:
                if e.args[0] == -202:
                    print("ERRO MQTT (-202): Versão do protocolo não aceita ou outro problema de conexão.")
                else:
                    print(f'Erro genérico MQTT: {e}')
                client = None
            except Exception as e:
                print(f'Erro inesperado ao conectar MQTT: {e}')
                client = None
        else:
            print('\nFalha ao conectar ao Wi-Fi.')

def mqtt_callback(topic, msg):
    print(f"Mensagem MQTT recebida: Tópico={topic.decode()}, Mensagem={msg.decode()}")
    if topic == MQTT_TOPIC_SUB:
        if msg.decode() == 'dispensar':
            print("Comando 'dispensar' recebido via MQTT.")
            servo_dispenser.dispense()
            buzzer.buzz(500)
        elif msg.decode() == 'bipe':
            print("Comando 'bipe' recebido via MQTT.")
            buzzer.buzz(100)

# --- Configuração dos Pinos da ESP32 ---
TRIG_PIN_ULTRASONIC = 4
ECHO_PIN_ULTRASONIC = 5
SERVO_MOTOR_PIN = 18
BUZZER_PIN = 21

# --- Inicialização dos Componentes ---
try:
    ultrasonic_sensor = UltrasonicSensor(TRIG_PIN_ULTRASONIC, ECHO_PIN_ULTRASONIC)
    servo_dispenser = ServoDispenser(SERVO_MOTOR_PIN)
    buzzer = Buzzer(BUZZER_PIN)
    print("Hardware inicializado com sucesso!")
except Exception as e:
    print(f"ERRO CRÍTICO: Falha ao inicializar hardware. Verifique os pinos e conexões. Erro: {e}")
    machine.reset()

# --- Variáveis de Controle ---
distance_threshold_cm = 15
last_action_time = 0
cooldown_s = 3
last_mqtt_publish_time = 0
mqtt_publish_interval_s = 5

print("Sistema pronto. Conectando ao Wi-Fi...")
connect_wifi()

# --- Loop Principal do Programa ---
while True:
    current_time = time.time()

    if client:
        try:
            client.check_msg()
        except OSError as e:
            print(f"Erro MQTT (check_msg): {e}. Tentando reconectar...")
            connect_wifi()

    # --- Leitura do Sensor Ultrassônico ---
    distance = ultrasonic_sensor.read_distance()

    if distance != -1 and distance > 0:
        if distance < distance_threshold_cm:
            if (current_time - last_action_time) > cooldown_s:
                print(f"Objeto a {distance:.2f} cm. Acionando dispensador e Buzzer!")
                servo_dispenser.dispense()
                buzzer.buzz(300)
                last_action_time = current_time

        if client and (current_time - last_mqtt_publish_time) > mqtt_publish_interval_s:
            try:
                msg = f"{distance:.2f}".encode()
                client.publish(MQTT_TOPIC_PUB, msg)
                print(f"MQTT Publicado: Tópico={MQTT_TOPIC_PUB.decode()}, Distância={msg.decode()} cm")
                last_mqtt_publish_time = current_time
            except OSError as e:
                print(f"Erro MQTT (publish): {e}. Tentando reconectar...")
                connect_wifi()
    else:
        print("Leitura de distância inválida (fora de alcance ou erro do sensor).")
        if client and (current_time - last_mqtt_publish_time) > mqtt_publish_interval_s:
            try:
                client.publish(MQTT_TOPIC_PUB, b"erro")
                print("MQTT Publicado: Erro na leitura de distância.")
                last_mqtt_publish_time = current_time
            except OSError as e:
                print(f"Erro MQTT (publish error): {e}. Tentando reconectar...")
                connect_wifi()

    time.sleep(0.1)
