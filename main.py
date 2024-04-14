##############################################
#
# TEMPHUM - SKR v0.32
#
# last update 14.04.2024
#
##############################################

import network
import secrets
import ubinascii
import ntptime
import time
from machine import Pin
from time import sleep
from time import time
import dht
from umqtt.simple import MQTTClient

# SENSOR CONTROL
SENSOR1 = True
SENSOR2 = True

# Main loop frequency in seconds
MAIN_FREQ = 0.2

# APPLICATION RUN PARAMETERS
# Sampling period in seconds
#SAMPL_PERIOD = 60
SAMPL_PERIOD = 300
#SAMPL_PERIOD = 5

# SAVE TO LOG
SAVE_TO_LOG = False

# Reconnect WIFI ticks count
WIFI_RECONNECT = 100

# WIFI AND MQTT PARAMETERS"
CLIENT_ID = ubinascii.hexlify(machine.unique_id())

# PINS DEFINITIONS
sensor1 = dht.DHT11(Pin(0))
sensor2 = dht.DHT11(Pin(4))
intled = machine.Pin("LED", machine.Pin.OUT)

wlan = network.WLAN(network.STA_IF)

class App_status():
    INITIATION = 0
    CONN_WIFI = 1
    WAIT_WIFI = 2
    CONN_MQTT = 3
    WAIT_MQTT = 4
    CONNECTED = 5
    WAIT_PUBL = 6
    PUBLISH   = 7
    
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

led_ctrl = LedControl()
app_stat = App_status.INITIATION
app_counter = 0
time_counter = time()

def log(input):
    s = "[" + str(app_counter) + "] " + input
    print(s)
    if SAVE_TO_LOG:
        f = open("temphum.log", "a")
        f.write(s+"\n")
        f.close()

def publish_to_mqtt():
    try:
       
        # SENSOR 1
        if SENSOR1:
            log("Sensor 1 Drewniak")
            sensor1.measure()
            temp = sensor1.temperature()
            hum = sensor1.humidity()        
            log("Drewniak Temperature: %3.1f C" %temp)
            log("Drewniak Humidity: %3.1f %%" %hum)
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_IN, str(temp).encode())
            log("[OK] MQTT published temperature")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_IN, str(hum).encode())
            log("[OK] MQTT published humidity")
        

        # SENSOR 2
        if SENSOR2:
            log("Sensor 2 Garden")
            sensor2.measure()
            temp = sensor2.temperature()
            hum = sensor2.humidity()
            log("Outside Temperature: %3.1f C" %temp)
            log("Outside Humidity: %3.1f %%" %hum)
            
            mqttClient.publish(secrets.MQTT_TOPIC_TEMP_OUT, str(temp).encode())
            log("[OK] MQTT published temperature")
            mqttClient.publish(secrets.MQTT_TOPIC_HUMID_OUT, str(hum).encode())
            log("[OK] MQTT published humidity")
            
        # Blink to confirm
        led_ctrl.blink()


    except OSError as e:
        log("[ER] Failed to read sensor.")
        

def connect_wifi():
    log("Connecting WIFI: " + secrets.WIFI_SSID)
    wlan.active(True)
    wlan.connect(secrets.WIFI_SSID, secrets.WIFI_PASS)


def set_global_time():
    log("Connecting NTP: " + secrets.NTP_SERVER)
    ntptime.host = secrets.NTP_SERVER
    try:
        ntptime.settime()
        log("Local time after synchronization：" + str(time.localtime()))
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

def controller():
    global app_counter
    global app_stat
    global time_counter
    global mqttClient
    
    global wlan
    
    # INIT
    if app_counter == 0:
        log("***  TEMPHUM   ***")
        log("v 0.32")
        log(" ")
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
                    #set_global_time()                    
                    app_stat = App_status.CONN_MQTT
                elif led_ctrl.counter > WIFI_RECONNECT:
                    app_stat = App_status.CONN_WIFI

            # WIFI CONNECTED
            elif app_stat == App_status.CONN_MQTT:
                log("Connecting MQTT: Client: " + str(CLIENT_ID) + " , Broker: " + secrets.MQTT_BROKER + " , Port: " + str(secrets.MQQT_PORT))

                mqttClient = MQTTClient(CLIENT_ID, secrets.MQTT_BROKER, secrets.MQQT_PORT, user=secrets.MQTT_USER, password=secrets.MQTT_PASS, keepalive=SAMPL_PERIOD+10)    
                mqttClient.connect()
                app_stat = App_status.WAIT_MQTT
            
            # CONNECT TO MQTT
            elif app_stat == App_status.WAIT_MQTT:
                app_stat = App_status.CONNECTED
                
            # MQTT CONNECTED
            elif app_stat == App_status.CONNECTED:
                log("[OK] MQTT Connected")
                app_stat = App_status.PUBLISH
                
            # WAITING TIME FOR PUBLISHING
            elif app_stat == App_status.WAIT_PUBL:
                if time() - time_counter > SAMPL_PERIOD:
                    if wlan.isconnected():
                        app_stat = App_status.PUBLISH
                    else:
                        log("[ER] Connectivity error. Reconnecting")
                        led_ctrl.reset_counter()
                        app_stat = App_status.CONN_WIFI
                    
            # PUBLISHING ACTION
            elif app_stat == App_status.PUBLISH:            
                publish_to_mqtt()
                app_stat = App_status.WAIT_PUBL
                time_counter = time()
        
        except OSError as e:
            log("[ER] Error : " + e.message + " " + e.args)
            led_ctrl.reset_counter()
            app_stat = App_status.CONN_WIFI
            

def main():
    global app_counter
        
    while True:
        sleep(MAIN_FREQ)
        status_light()
        controller()
        
        app_counter += 1
        
main()



# REFERENCES
# https://alanedwardes.com/blog/posts/pico-home-assistant-motion-temperature-sensor/
# https://www.donskytech.com/umqtt-simple-micropython-tutorial/
# https://mpython.readthedocs.io/en/v2.2.1/library/mPython/umqtt.simple.html
# https://randomnerdtutorials.com/micropython-mqtt-esp32-esp8266/
# https://opensource.com/article/21/2/home-assistant-custom-sensors
