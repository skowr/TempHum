##############################################
#
# TEMPHUM - SKR v0.38
#
# last update 3.07.2024
#
##############################################

import network
import secrets
import ubinascii
import ntptime
import os
import time
from machine import Pin
from machine import reset
from time import sleep
from time import time
from time import localtime
import dht
from umqtt.simple import MQTTClient


class GLOBAL_CONSTANTS:

    # Main loop frequency in seconds
    MAIN_FREQ = 0.2
    
    # APPLICATION RUN PARAMETERS
    # Sampling period in seconds
    #SAMPL_PERIOD = 60
    SAMPL_PERIOD = 300
    #SAMPL_PERIOD = 5
    
    # Reconnect WIFI ticks count
    WIFI_RECONNECT = 100


    # SENSOR CONTROL
    SENSOR_OUT = True # Outside Garden
    SENSOR_IN = True # Inside
    
    # SAVE TO LOG
    SAVE_TO_LOG = True
    # Max KB size of a log file
    MAX_LOG_SIZE = 500
    # Log file name
    LOG_FILENAME = "temphum.log"        
    

class App_status:
    INITIATION = 0
    CONN_WIFI = 1
    WAIT_WIFI = 2
    SYNC_TIME = 3
    CONN_MQTT = 4
    WAIT_MQTT = 5
    CONNECTED = 6
    WAIT_PUBL = 7
    READ_SENS = 8
    PUBLISH   = 9

    
class LedControl:
    ledstatus = False
    counter = 0
    
    def reset_counter(self):
        self.counter = 0
        self.ledstatus = False
        
    def toggle(self):
        self.ledstatus = not self.ledstatus
        
    def blink(self):
        for i in range(5):
            led_ctrl.ledstatus = not led_ctrl.ledstatus
            sleep(0.1)
            intled.value(led_ctrl.ledstatus)
            
        led_ctrl.ledstatus = self.ledstatus
        intled.value(led_ctrl.ledstatus)
        

class DHTSensor:
    temp = 0
    hum = 0
    read = False


# INIT PROGRAM        

# WIFI AND MQTT PARAMETERS"
CLIENT_ID = ubinascii.hexlify(machine.unique_id())

# PINS DEFINITIONS
sensor1 = dht.DHT11(Pin(0))
sensor2 = dht.DHT11(Pin(4))
intled = machine.Pin("LED", machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)

led_ctrl = LedControl()
app_stat = App_status.INITIATION
app_counter = 0
time_counter = time()

dhtSensIn = DHTSensor()
dhtSensOut = DHTSensor()

def file_exists(filename):
    try:
        os.stat(filename)
        return True
    except OSError:
        return False

def log(input):
    global GLOBAL_CONSTANTS
    

    t = localtime()
    u = str(f"{t[0]}.{t[1]}.{t[2]} {t[3]}:{t[4]}:{t[5]}")

    s = "[" + u + " | " + str(app_counter) + "] " + input
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


def read_sensors():
    global GLOBAL_CONSTANTS
    
    # SENSOR 1 OUTSIDE
    if GLOBAL_CONSTANTS.SENSOR_OUT:
        try:
            log("Sensor 1 Garden")
            sensor1.measure()
            dhtSensOut.temp = sensor1.temperature()
            dhtSensOut.hum = sensor1.humidity()
            dhtSensOut.read = True
            log("Drewniak Temperature: %3.1f C" %dhtSensOut.temp)
            log("Drewniak Humidity: %3.1f %%" %dhtSensOut.hum)

        except OSError as e:
            log("[ER] Failed to read sensor. : " + str(e.errno))
            dhtSensOut.read = False
            
    # SENSOR 2 INSIDE
    if GLOBAL_CONSTANTS.SENSOR_IN:

        try:
            log("Sensor 2 Drewniak")
            sensor2.measure()
            dhtSensIn.temp = sensor2.temperature()
            dhtSensIn.hum = sensor2.humidity()
            dhtSensIn.read = True
            log("Outside Temperature: %3.1f C" %dhtSensIn.temp)
            log("Outside Humidity: %3.1f %%" %dhtSensIn.hum)
                        
        except OSError as e:
            log("[ER] Failed to read sensor. : " + str(e.errno))
            dhtSensIn.read = False
            
    return dhtSensIn.read or dhtSensOut.read

def publish_sensors():
    try:
        if GLOBAL_CONSTANTS.SENSOR_OUT and dhtSensOut.read:
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_OUT, str(dhtSensOut.temp).encode())
            log("[OK] MQTT published temperature sensor 1 out")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_OUT, str(dhtSensOut.hum).encode())
            log("[OK] MQTT published humidity sensor 1 out")
            
        if GLOBAL_CONSTANTS.SENSOR_IN and dhtSensIn.read:
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_IN, str(dhtSensIn.temp).encode())
            log("[OK] MQTT published temperature sensor 2 in")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_IN, str(dhtSensIn.hum).encode())
            log("[OK] MQTT published humidity sensor 2 in")
        
        # Blink to confirm
        led_ctrl.blink()

    except OSError as e:
        log("[ER] Failed to publish sensor. : " + str(e.errno))
        return False
    return True

def connect_wifi():
    log("Connecting WIFI: " + secrets.WIFI_SSID)
    wlan.active(True)
    wlan.connect(secrets.WIFI_SSID, secrets.WIFI_PASS)

def set_global_time():
    try:
        log("Connecting NTP: " + secrets.NTP_SERVER)
        ntptime.host = secrets.NTP_SERVER
        t = localtime()
        log("Local time after synchronization：" + str(f"{t[0]}.{t[1]}.{t[2]} {t[3]}:{t[4]}:{t[5]}"))
    except:
        log("[ER] Cannot connect time server")


def status_light():
    global app_counter
    global app_stat
    
    if app_stat == App_status.INITIATION:
        led_ctrl.ledstatus = not led_ctrl.ledstatus
        
    if app_stat == App_status.WAIT_WIFI:
        led_ctrl.ledstatus = ((led_ctrl.counter / 2) % 2)
                
    if app_stat == App_status.WAIT_PUBL:
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
    global mqttClient    
    global wlan
    
    global GLOBAL_CONSTANTS
    
    # INIT
    if app_counter == 0:
        log("***  TEMPHUM   ***")
        log("v 0.38")
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
                log("Connecting MQTT: Client: " + str(CLIENT_ID) + " , Broker: " + secrets.MQTT_BROKER + " , Port: " + str(secrets.MQQT_PORT))

                mqttClient = MQTTClient(CLIENT_ID, secrets.MQTT_BROKER, secrets.MQQT_PORT, user=secrets.MQTT_USER, password=secrets.MQTT_PASS, keepalive=GLOBAL_CONSTANTS.SAMPL_PERIOD+10)    
                mqttClient.connect()
                
                log("Waiting for connection")
                app_stat = App_status.WAIT_MQTT
                            
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
                    
            # READING SENSORS
            
            elif app_stat == App_status.READ_SENS:
                if read_sensors():
                    # publish
                    app_stat = App_status.PUBLISH
                    
            
            # PUBLISHING ACTION
            elif app_stat == App_status.PUBLISH:
                if publish_sensors():
                    app_stat = App_status.WAIT_PUBL
                    time_counter = time()
                else:
                    led_ctrl.reset_counter()
                    app_stat = App_status.CONN_WIFI
                    
        
        except OSError as e:
            log("[ER] Error : " + str(e.errno))            
            led_ctrl.reset_counter()
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

# TODO
# CSV Saving to disk
# Refactoring and error handling

# REFERENCES
# https://alanedwardes.com/blog/posts/pico-home-assistant-motion-temperature-sensor/
# https://www.donskytech.com/umqtt-simple-micropython-tutorial/
# https://mpython.readthedocs.io/en/v2.2.1/library/mPython/umqtt.simple.html
# https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
# https://opensource.com/article/21/2/home-assistant-custom-sensors
