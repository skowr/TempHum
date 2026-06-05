##############################################
#
# TEMPHUM - SKR v0.6
#
# last update 05.06.2026
#
##############################################

import network
import secrets
import ubinascii
import ntptime
import os
import time
import bme280_float as bme280
import dht
import network
import mrequests as requests
from machine import Pin
from machine import reset
from time import sleep
from time import time
from time import localtime
from umqtt.simple import MQTTClient


class GLOBAL_CONSTANTS:
    
    PROGRAM_VERSION = "TEMPHUM - SKR v0.6"

    # Main loop frequency in seconds
    MAIN_FREQ = 0.2
    
    # APPLICATION RUN PARAMETERS
    # Sampling period in seconds
    #SAMPL_PERIOD = 60
    SAMPL_PERIOD = 300
    #SAMPL_PERIOD = 5

    # Reset period
    # RESET_PERIOD = 0  # don't reset
    RESET_PERIOD = 60*60*24*2 # 2 days

    # Reconnect WIFI ticks count
    WIFI_RECONNECT = 100

    # Sensors control
    SENSOR_OUT = True # Outside Garden
    SENSOR_IN = True # Inside
    
    # Sensors calibration
    SENSOR_OUT_TEMP_CAL = 0
    SENSOR_OUT_HUM_CAL = 0
    SENSOR_OUT_PRESS_CAL = 0
    SENSOR_IN_TEMP_CAL = -1.3
    SENSOR_IN_HUM_CAL = -8.0
    
    # SAVE TO LOG
    SAVE_TO_LOG = True
    # Max KB size of a log file
    MAX_LOG_SIZE = 500
    # Log file name
    LOG_FILENAME = "temphum.log"
    
    # Work without constant blinking
    DARK_MODE = True
    #DARK_MODE = False
    
class App_status:
    INITIATION = 0	# Startup
    CONN_WIFI = 1	# Establish connection to WiFi
    WAIT_WIFI = 2	# Wait for WIFI configuration
    SYNC_TIME = 3	# Try synchronize time
    CONN_MQTT = 4	# Establish connection to MQTT
    WAIT_MQTT = 5	# Wait for MQTT
    CONNECTED = 6	# Fake status after MQTT configuration
    WAIT_PUBL = 7	# Wait for sensor read
    READ_SENS = 8	# Read Sensors action
    PUBLISH   = 9	# Publish Sensors results action

    
class LedControl:
    ledstatus = False
    counter = 0
    
    def reset_counter(self):
        self.counter = 0
        self.ledstatus = False
        
    def toggle(self):
        self.ledstatus = not self.ledstatus
        
    def blink(self):
        for i in range(10):
            led_ctrl.ledstatus = not led_ctrl.ledstatus
            sleep(0.1)
            intled.value(led_ctrl.ledstatus)        
        led_ctrl.ledstatus = self.ledstatus
        intled.value(led_ctrl.ledstatus)
        
    def error_blink(self):
        for i in range(20):
            led_ctrl.ledstatus = not led_ctrl.ledstatus
            sleep(0.05)
            intled.value(led_ctrl.ledstatus)
        
        led_ctrl.ledstatus = self.ledstatus
        intled.value(led_ctrl.ledstatus)
        

class DHTSensor:
    temp = 0 # Temperature
    hum = 0 # Humuidity
    read = False

class BMESensor:
    temp = 0 # Temperature
    hum = 0 # Humuidity
    press = 0 # Pressure
    read = False
    

# INIT PROGRAM        

# WIFI AND MQTT PARAMETERS
CLIENT_ID = ubinascii.hexlify(machine.unique_id())

# PINS DEFINITIONS
sensor_in = dht.DHT22(Pin(4))

i2c = machine.I2C(1, scl=machine.Pin(7), sda=machine.Pin(6), freq=400000)
sensor_bme = bme280.BME280(i2c=i2c)

intled = machine.Pin("LED", machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)

led_ctrl = LedControl()
app_stat = App_status.INITIATION
app_counter = 0
time_counter = time()
reset_counter = time()

dhtSensIn = DHTSensor()
#dhtSensOut = DHTSensor()
bmeSensOut = BMESensor()

def file_exists(filename):
    try:
        os.stat(filename)
        return True
    except OSError:
        return False

def log(input):
    global GLOBAL_CONSTANTS
    

    t = localtime()
    u = str(f"{t[0]}.{t[1]:02d}.{t[2]:02d} {t[3]:02d}:{t[4]:02d}:{t[5]:02d}")

    s = "[" + u + " | " + str(f"{app_counter:06d}") + "] " + input
    print(s)
    if GLOBAL_CONSTANTS.SAVE_TO_LOG:        
        size = 0
        if file_exists(GLOBAL_CONSTANTS.LOG_FILENAME):
            stats = os.stat(GLOBAL_CONSTANTS.LOG_FILENAME)
            size = stats[6]
            
        if size < GLOBAL_CONSTANTS.MAX_LOG_SIZE*1024:
            f = open(GLOBAL_CONSTANTS.LOG_FILENAME, "a")
            f.write(s+"\n")
            f.close()
        else:
            os.remove(GLOBAL_CONSTANTS.LOG_FILENAME)

def url_encode(string):
    encoded_string = ''
    for char in string:
        if char.isalpha() or char.isdigit() or char in '-._~':
            encoded_string += char
        else:
            encoded_string += '%' + '{:02X}'.format(ord(char))
    return encoded_string

def read_sensors():
    global GLOBAL_CONSTANTS
    
    # SENSOR 1 OUTSIDE
    if GLOBAL_CONSTANTS.SENSOR_OUT:
        try:
            log("[INF] Reading sensor 1 Outside")
            bmeSensOut.temp, bmeSensOut.press, bmeSensOut.hum = sensor_bme.read_compensated_data()
            bmeSensOut.temp = round(bmeSensOut.temp, 1) + GLOBAL_CONSTANTS.SENSOR_OUT_TEMP_CAL
            bmeSensOut.press = round(bmeSensOut.press / 100, 1)
            bmeSensOut.hum = round(bmeSensOut.hum, 1) + GLOBAL_CONSTANTS.SENSOR_OUT_HUM_CAL
            bmeSensOut.read = True
            log("[OK] Outside Temperature: %3.1f C" %bmeSensOut.temp)
            log("[OK] Outside Humidity: %3.1f %%" %bmeSensOut.hum)
            log("[OK] Outside Pressure: %3.1f hPa" %bmeSensOut.press)

        except OSError as e:
            log("[ER] Failed to read outside sensor. : " + str(e.errno))
            bmeSensOut.read = False
            
    # SENSOR 2 INSIDE
    if GLOBAL_CONSTANTS.SENSOR_IN:

        try:
            log("[INF] Reading sensor 2 Inside")
            sensor_in.measure()
            dhtSensIn.temp = sensor_in.temperature() + GLOBAL_CONSTANTS.SENSOR_IN_TEMP_CAL
            dhtSensIn.hum = sensor_in.humidity() + GLOBAL_CONSTANTS.SENSOR_IN_HUM_CAL
            dhtSensIn.read = True
            log("[OK] Inside Temperature: %3.1f C" %dhtSensIn.temp)
            log("[OK] Inside Humidity: %3.1f %%" %dhtSensIn.hum)
                        
        except OSError as e:
            log("[ER] Failed to read inside sensor. : " + str(e.errno))
            dhtSensIn.read = False
            
    return dhtSensIn.read or bmeSensOut.read

def mqtt_publish_sensors():
    try:
        if GLOBAL_CONSTANTS.SENSOR_OUT and bmeSensOut.read:
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_OUT, str(bmeSensOut.temp).encode())
            log("[OK] MQTT published temperature sensor 1 (Outside)")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_OUT, str(bmeSensOut.hum).encode())
            log("[OK] MQTT published humidity sensor 1 (Outside)")
            mqttClient.publish(secrets.MQTT_TOPIC_PRESS_OUT, str(bmeSensOut.press).encode())
            log("[OK] MQTT published pressure sensor 1 (Outside)")
        else:
            log("[WRN] No data to publish on sensor 1 (Outside)")
            
        if GLOBAL_CONSTANTS.SENSOR_IN and dhtSensIn.read:
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_IN, str(dhtSensIn.temp).encode())
            log("[OK] MQTT published temperature sensor 2 (Inside)")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_IN, str(dhtSensIn.hum).encode())
            log("[OK] MQTT published humidity sensor 2 (Inside)")
        else:
            log("[WRN] No data to publish on sensor 2 (Inside)")
        
        # Blink to confirm
        led_ctrl.blink()

    except OSError as e:
        log("[ER] Failed to publish sensor. : " + str(e.errno))
        return False
    return True

def quest_publish_sensors():
    query = f"INSERT INTO temphumpres(timestamp,internal_temp,internal_humidity,external_temp,external_humidity,external_pressure) VALUES(now(),{str(dhtSensIn.temp)},{str(dhtSensIn.hum)},{str(bmeSensOut.temp)},{str(bmeSensOut.hum)},{str(bmeSensOut.press)})"
    
    log("[INF] QuestDB Query: " + query)
    full_url = secrets.QUESTDB_ADDR + "/exec?query="+url_encode(query)
    log("[INF] QuestDB API Call: " + full_url)
    
    try:
        requests.get(url=full_url, auth=(secrets.QUESTDB_USER, secrets.QUESTDB_PASS))
        log("[OK] QuestDB reading published")
        return True
    except Exception as error:
        if str(error) == "Unsupported types for __add__: 'str', 'bytes'":
            return True
        else:
            print("[ERR] General error: ",str(error))
            return False    

def connect_wifi():
    log("[INF] Connecting WIFI: " + secrets.WIFI_SSID)
    wlan.active(True)
    wlan.connect(secrets.WIFI_SSID, secrets.WIFI_PASS)

def set_global_time():
    try:
        log("[INF] Connecting NTP: " + secrets.NTP_SERVER)
        ntptime.host = secrets.NTP_SERVER
        t = localtime()
        log("[OK] Local time after synchronization：" + str(f"{t[0]}.{t[1]}.{t[2]} {t[3]}:{t[4]}:{t[5]}"))
    except:
        log("[ER] Cannot connect time server")


def connect_mqtt():
    global mqttClient
    
    try:
    
        log("[INF] Connecting MQTT: Client: " + str(CLIENT_ID) + " , Broker: " + secrets.MQTT_BROKER + " , Port: " + str(secrets.MQQT_PORT))

        mqttClient = MQTTClient(CLIENT_ID, secrets.MQTT_BROKER, secrets.MQQT_PORT, user=secrets.MQTT_USER, password=secrets.MQTT_PASS, keepalive=GLOBAL_CONSTANTS.SAMPL_PERIOD+10)    
        mqttClient.connect()
        
        return True

    except:
        log("[ER] Cannot connect to MQTT")
        
        return False


def status_light():
    global app_counter
    global app_stat
    
    if app_stat == App_status.INITIATION:
        led_ctrl.ledstatus = not led_ctrl.ledstatus
        
    if app_stat == App_status.WAIT_WIFI:
        led_ctrl.ledstatus = ((led_ctrl.counter / 2) % 2)
                
    if app_stat == App_status.WAIT_PUBL:
        if GLOBAL_CONSTANTS.DARK_MODE:
            led_ctrl.ledstatus = False
        else:
            led_ctrl.ledstatus = ((led_ctrl.counter / 6) % 2)
        
    if app_stat == App_status.PUBLISH:        
        led_ctrl.ledstatus = False
        

    intled.value(led_ctrl.ledstatus)

    led_ctrl.counter += 1
    
def get_free_space():
    stat = os.statvfs('/')
    freespace = stat[0] * stat[4] / 1024
    
    return freespace

def controller():
    global app_counter
    global app_stat
    global time_counter
    global reset_counter
    global mqttClient    
    global wlan
    
    global GLOBAL_CONSTANTS
    
    # INIT
    if app_counter == 0:
        log("***  TEMPHUMPRES   ***")
        log(GLOBAL_CONSTANTS.PROGRAM_VERSION)
        log("Space avail : " + str(get_free_space()) + " KB")
        
        app_stat = App_status.INITIATION
    elif app_counter < 10:
        app_stat = App_status.INITIATION    
    elif app_counter == 10:
        app_stat = App_status.CONN_WIFI
    else:
        
        try:
        
            # CONNECT TO WIFI
            if app_stat == App_status.CONN_WIFI:
                led_ctrl.reset_counter()
                connect_wifi()
                app_stat = App_status.WAIT_WIFI
            
            # WAIT FOR WIFI CONNECTION 
            elif app_stat == App_status.WAIT_WIFI:
                # ... AND CONNECT MQTT
                if wlan.isconnected():
                    log("[OK] WiFi connected: " + str(wlan.ifconfig()))                    
                    app_stat = App_status.SYNC_TIME
                elif led_ctrl.counter > GLOBAL_CONSTANTS.WIFI_RECONNECT:
                    app_stat = App_status.CONN_WIFI

            # SYNC GLOBAL TIME            
            elif app_stat == App_status.SYNC_TIME:
                set_global_time()
                app_stat = App_status.CONN_MQTT            
            
            # CONNECT TO MQTT
            elif app_stat == App_status.CONN_MQTT:
                if connect_mqtt():
                
                    log("[INF] Waiting for connection")
                    app_stat = App_status.WAIT_MQTT
                else:
                    log("[ER] MQTT not connected. Trying to reconnect")
                    time_counter = time()
                    led_ctrl.error_blink()
                            
                        
            # CONNECTED TO MQTT
            elif app_stat == App_status.WAIT_MQTT:
                app_stat = App_status.CONNECTED
                
            # MQTT CONNECTED
            elif app_stat == App_status.CONNECTED:
                log("[OK] MQTT Connected")
                app_stat = App_status.READ_SENS
                
            # WAITING TIME FOR PUBLISHING
            elif app_stat == App_status.WAIT_PUBL:
                if time() - time_counter > GLOBAL_CONSTANTS.SAMPL_PERIOD:
                    if wlan.isconnected():
                        app_stat = App_status.READ_SENS
                    else:
                        log("[ER] Connectivity error. Reconnecting")
                        led_ctrl.reset_counter()
                        app_stat = App_status.CONN_WIFI
                        time_counter = time()
                    
            # READING SENSORS            
            elif app_stat == App_status.READ_SENS:
                if read_sensors():
                    # publish
                    app_stat = App_status.PUBLISH
                else:
                    # Failed to read sensors
                    log("[ER] Cannot read sensors. Attempting again in next iteration")
                    led_ctrl.reset_counter()
                    led_ctrl.error_blink()
                    app_stat = App_status.WAIT_PUBL
                    time_counter = time()
                   
            
            # PUBLISHING ACTION
            elif app_stat == App_status.PUBLISH:
                if mqtt_publish_sensors():
                    app_stat = App_status.WAIT_PUBL
                else:
                    # Failed to publish.
                    log("[ER] Publishing error. Retrying to reconnect to MQTT and publish later")
                    led_ctrl.reset_counter()
                    app_stat = App_status.CONN_MQTT

                quest_publish_sensors()

                time_counter = time()
                
            # RESET ACTION
            if GLOBAL_CONSTANTS.RESET_PERIOD > 0:
                if time() - reset_counter > GLOBAL_CONSTANTS.RESET_PERIOD:
                    log("[INF] *** Reset Action ***")
                    machine.reset()                            
        
        except OSError as e:
            log("[ER] Error : " + str(e.errno))            
            led_ctrl.reset_counter()
            led_ctrl.error_blink()
            time_counter = time()
            app_stat = App_status.CONN_WIFI
            

def main():
    global app_counter
    global GLOBAL_CONSTANTS
        
    while True:
        sleep(GLOBAL_CONSTANTS.MAIN_FREQ)
        status_light()
        controller()
        
        app_counter += 1
        
main()
