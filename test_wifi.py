import network
import secrets
import time


def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(secrets.WIFI_SSID, secrets.WIFI_PASS)
    time.sleep(5)
    print(wlan.isconnected())
    

def scan_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    
    networks = wlan.scan()
    
    if networks:
        print("Available WiFi Networks:")
        for ssid, bssid, channel, rssi, authmode, hidden in networks:
            print("- SSID:", ssid.decode(), "| BSSID:", ":".join("{:02x}".format(x) for x in bssid), "| RSSI:", rssi, "dBm", "| Channel:", channel, "| Auth Mode:", authmode)
    else:
        print("No WiFi networks found.")


#connect_wifi()
scan_wifi()