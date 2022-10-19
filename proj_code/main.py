#------------------------------------------------------------------------------#
#                                                                              #
#                           Developed by Saúl Carvalho                         #
#                                                                              #
#                           Ano: 2021 / 2022                                   #
#                                                                              #
#------------------------------------------------------------------------------#
import pycom
import machine
import time
import socket
import ubinascii
import binascii
import struct
import math
from machine     import Pin
from machine     import I2C
from network     import WLAN
from network     import LoRa
from network     import Bluetooth
from simple      import MQTTClient
from LIS2HH12    import LIS2HH12
from LTR329ALS01 import LTR329ALS01
from MPL3115A2   import MPL3115A2,ALTITUDE,PRESSURE

# SI7006A20 temperature function
def temp():
    SI7006A20.writeto(SI7006A20_ADDR, bytearray([0xF3]))                         # 0xF3 -> Read Temperature, Hold Master Mode
    time.sleep(0.5)
    data = SI7006A20.readfrom(SI7006A20_ADDR, 3)
    data = ((data[0] & 0xFF) << 8) + (data[1] & 0xFF)
    temp = ((175.72 * data) / 65536.0) - 46.85
    return temp

# SI7006A20 humidity function
def humi():
    SI7006A20.writeto(SI7006A20_ADDR, bytearray([0xF5]))                         # 0xF5 -> Read Humidity, Hold Master Mode
    time.sleep(0.5)
    data = SI7006A20.readfrom(SI7006A20_ADDR, 2)
    data = ((data[0] & 0xFF) << 8) + (data[1] & 0xFF)
    humi = ((125.0 * data) / 65536.0) - 6.0
    return humi

# Round up function
def round_up(n, decimals=0):
    multiplier = 10 ** decimals
    return math.ceil(n * multiplier) / multiplier

# MQTT SUBSCRIBE CALLBACK
def subscribe_mqtt_cb(topic, msg):
   global modeMQTT
   global cayenneActuator
   print("\n>MQTT_PUB: Publishing in MQTT topic:")
   #print("", msg)                                                               # uncomment for MQTT data
   read = msg.decode('UTF-8')                                                    # Converte byte stream to string
   x = read.split(",")
   if (int(x[1]) == 1):
       print(">MQTT_CMD: Data sent by MQTT in fast mode \t\t-> FAST MQTT")
       modeMQTT = 1
       cayenneActuator = x[0]
       for j in range(0,5, 1):
           for i in range(0, 255, 1):
               time.sleep_ms(1)
               a = 0x00
               y = hex(a << 16 | a << 8 | i)
               pycom.rgbled(int(y))
           for i in range(255, 0, -1):
               time.sleep_ms(1)
               a = 0x00
               y = hex(a << 16 | a << 8 | i)
               pycom.rgbled(int(y))
   else:
       print(">MQTT_CMD: Data sent by MQTT in slow mode \t\t-> SLOW MQTT")
       modeMQTT = 0
       cayenneActuator = x[0]
       for j in range(0,2, 1):
           for i in range(0, 255, 1):
               time.sleep_ms(5)
               a = 0x00
               y = hex(a << 16 | a << 8 | i)
               pycom.rgbled(int(y))
           for i in range(255, 0, -1):
               time.sleep_ms(5)
               a = 0x00
               y = hex(a << 16 | a << 8 | i)
               pycom.rgbled(int(y))

# LORAWAN DOWNLINK CALLBACK
def lora_cb(lora):
    global modeLORA

    events = lora.events()
    if events & LoRa.RX_PACKET_EVENT:
        if lora_socket is not None:
            frame, port = lora_socket.recvfrom(192)                              # longest frame is +-220

        print("\n>LORAWAN_DL: LoRaWAN downlink message received!")
        print(">LORAWAN_DL: Showing LoRa reception data (RX).")
        print(" RSSI: " + "\t\t\t" + str(lora.stats().rssi) + "\t dBm")
        print(" SNR: "  + "\t\t\t" + str(lora.stats().snr)  + "\t dB", end="")
        if lora.stats().snr > 0:
            print("\t\t\t\t-> SIGNAL ABOVE NOISE LEVEL")
        elif lora.stats().snr < 0:
            print("\t\t\t\t-> SIGNAL AT NOISE LEVEL")
        else:
            print("\t\t\t\t-> SIGNAL BELOW NOISE LEVEL")
        print(" Data Rate (DR): " + "\t" + str(lora.stats().sfrx))

        if (frame[2] == 100):                                                    # Decode Cayenne Downklink message - 3rd byte is button state
            print(">LORAWAN_CMD: Data sent by LoRaWAN in fast mode \t-> FAST LORA")
            modeLORA = 1
            for j in range(0,5, 1):
                for i in range(0, 255, 1):
                    time.sleep_ms(1)
                    a = 0x00
                    y = hex(i << 16 | a << 8 | a)
                    pycom.rgbled(int(y))
                for i in range(255, 0, -1):
                    time.sleep_ms(1)
                    a = 0x00
                    y = hex(i << 16 | a << 8 | a)
                    pycom.rgbled(int(y))
        elif (frame[2] == 0):
            print(">LORAWAN_CMD: Data sent by LoRaWAN in slow mode \t\t-> SLOW LORA")
            modeLORA = 0
            for j in range(0,2, 1):
                for i in range(0, 255, 1):
                    time.sleep_ms(5)
                    a = 0x00
                    y = hex(i << 16 | a << 8 | a)
                    pycom.rgbled(int(y))
                for i in range(255, 0, -1):
                    time.sleep_ms(5)
                    a = 0x00
                    y = hex(i << 16 | a << 8 | a)
                    pycom.rgbled(int(y))

    if events & LoRa.TX_PACKET_EVENT:
        print(">LORAWAN_UL: Showing LoRa transmission data (TX).")
        print(" Time on Air (ToA): "   + "\t" + str(lora.stats().tx_time_on_air) + "\t ms")
        print(" Nº of sent packets: " + "\t" + str(lora.stats().tx_counter + 1) + "\t packets")
        print(" Data Rate (DR): "  + "\t" + str(lora.stats().sftx))

################################################################################
print("\n>SYSTEM_CMD: Waking up LoPy \t\t\t\t\t-> WAKE UP")
# START CODE
pycom.heartbeat(False)
for i in range(0, 255, 1):
     time.sleep_ms(1)
     a = 0x00
     y = hex(i << 16 | a << 8 | i)
     pycom.rgbled(int(y))

modeMQTT = 0                                                                     # LED state for MQTT | 0 = slowMQTT, 1 = fastMQTT
modeLORA = 0
isWifiOk = 0
last_state = 0                                                                   # LED state for BLE
cayenneActuator = 0
doMQTT = 0                                                                       # flag to decide if data is pushed through LoRAWAN or MQTT
tSLEEP = 90000 # 90s = 1m30s

# CONST AND VAR FOR MQTT RX AND TX CYCLE
pT_TX_MQTT  = 0
int_TX_MQTT = 60000 #60s
pT_RX_MQTT  = 0
int_RX_MQTT = 1000  #1s
# CONST AND VAR FOR LORA TX CYCLE
pT_TX_LORA  = 0
int_TX_LORA = 60000 #60s
# CONST AND VAR FOR BLE RX CYCLE
pT_RX_BLE  = 0
int_RX_BLE = 10000 #10s
# CONST AND VAR FOR LED BLINK - RUN MODE
pT_LED  = 0
int_LED = 30000 #30s
# CONST AND VAR FOR WIFI
pT_WiFi  = 0
int_WiFi = 20000 #20s

# I2C CONFIG
li   = LTR329ALS01(None, sda = 'P22', scl = 'P21', gain = LTR329ALS01.ALS_GAIN_1X, integration = LTR329ALS01.ALS_INT_100, rate = LTR329ALS01.ALS_RATE_500)
alt  = MPL3115A2  (None, sda = 'P22', scl = 'P21', mode=ALTITUDE)
pres = MPL3115A2  (None, sda = 'P22', scl = 'P21', mode=PRESSURE)
SI7006A20_ADDR = const(0x40)                                                     # SI7006A20 address
SI7006A20      = I2C(0, I2C.MASTER, pins=('P22', 'P21'))

# BLE CONFIG
bt = Bluetooth()

# CONNECT TO WIFI NETWORK
wlan = WLAN(mode=WLAN.STA)
ssid     = 'NETWORK_SSID'
password = 'NETWORK_PASS'
wlan.connect(ssid, auth=(WLAN.WPA2, password))

join_wait = 0
print('\n>SYSTEM_CMD: Trying to connect to the WiFi network:')
print('>', end="")
while True:
    time.sleep(0.25)
    if not wlan.isconnected():
        print('.', end="")
        join_wait += 1
        if join_wait == 50:
            print("\n>SYSTEM_CMD: Connecting to the WiFi network", ssid , ",failed!")
            isWifiOk = 0
            join_wait = 0
            break
    else:
        print("\n>SYSTEM_CMD: Connecting to the WiFi network", ssid , ",success!")
        print(">WIFI_CMD: WiFi network data.")
        print(" Device IP: \t", wlan.ifconfig()[0])
        print(" Network IP: \t", wlan.ifconfig()[1])
        print(" Gateway IP: \t"  , wlan.ifconfig()[2])
        print(" DNS Server: \t", wlan.ifconfig()[3])

        # MQTT BROKER CONFIG
        broker = "mqtt.mydevices.com"
        client = MQTTClient("MQTT_CODE_BROKER", broker, user="MQTT_CODE_USER", password="MQTT_CODE_PASSWORD", port=1883)
        client.set_callback(subscribe_mqtt_cb)
        client.connect()
        client.subscribe(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/cmd/0") # cayenne button
        isWifiOk = 1
        join_wait = 0
        break

# LORAWAN CONFIG
lora = LoRa(mode=LoRa.LORAWAN, region=LoRa.EU868, adr=True, tx_retries=3, device_class=LoRa.CLASS_C)
lora.callback(trigger=(LoRa.RX_PACKET_EVENT | LoRa.TX_PACKET_EVENT), handler=lora_cb)

# Create an OTA authentication params
dev_eui = binascii.unhexlify('DEVICE_EUI')
app_eui = binascii.unhexlify('APPLICATION_EUI')
app_key = binascii.unhexlify('APPLICATION_KEY')

# LORA CHANNEL CONFIG
for channel in range(0, 8):
    lora.remove_channel(channel)
lora.add_channel(0, frequency=868500000, dr_min=0, dr_max=4)
lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)     # join a network using OTAA
join_wait = 0                                                                    # wait until the module has joined the network

# LORA SOCKET
lora_socket = socket.socket(socket.AF_LORA, socket.SOCK_RAW)                     # set the LoRaWAN data rate
lora_socket.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)                         # set the LoRaWAN data rate -> 5 - SF7
lora_socket.setblocking(True)

for i in range(255, 0, -1):                                                      # all networking configs done
     time.sleep_ms(1)
     a = 0x00
     y = hex(i << 16 | a << 8 | i)
     pycom.rgbled(int(y))

while True:
    # Cayenne MQTT button state - send mode
    if modeMQTT == 0:
        int_TX_MQTT = 60000 #60s -> Slow MQTT
    elif modeMQTT == 1:
        int_TX_MQTT = 30000 #30s -> Fast MQTT

    # LoRAWAN send mode
    if modeLORA == 0:
        int_TX_LORA = 60000 #60s -> Slow LORA
    elif modeLORA == 1:
        int_TX_LORA = 30000 #30s -> Fast LORA

    # TX - LORAWAN UPLINK
    cT_TX_LORA = int(time.time() * 1000)
    if ((cT_TX_LORA - pT_TX_LORA) >= int_TX_LORA):
        pT_TX_LORA = cT_TX_LORA

        # CONNECT TO LORAWAN GATEWAY
        if not lora.has_joined():
            for channel in range(0, 8):
                lora.remove_channel(channel)
            lora.add_channel(0, frequency=868500000, dr_min=0, dr_max=4)
            lora.join(activation=LoRa.OTAA, auth=(dev_eui, app_eui, app_key), timeout=0)     # join a network using OTAA
            # LORA SOCKET
            lora_socket = socket.socket(socket.AF_LORA, socket.SOCK_RAW)                     # set the LoRaWAN data rate
            lora_socket.setsockopt(socket.SOL_LORA, socket.SO_DR, 5)                         # set the LoRaWAN data rate -> 5 - SF7
            lora_socket.setblocking(True)
        join_wait = 0
        print('\n>SYSTEM_CMD: Trying to send data by LoRa:')
        print('>', end="")
        while True:
            time.sleep(0.25)
            if not lora.has_joined():
                print('.', end="")
                join_wait += 1
                if join_wait == 50:
                    print("\n>SYSTEM_CMD: Connection to the LoRaWAN gateway failed! Trying to send data by MQTT!")
                    doMQTT = 1
                    join_wait = 0
                    break
            else:
                print("\n>SYSTEM_CMD: Connection to the LoRaWAN gateway with sucess! Sending data by LoRa!")
                doMQTT = 0
                join_wait = 0
                break

        # PREPARE VARIABLES FOR LORAWAN UPLINK
        if doMQTT == 0: # connection to LoRaWAN gateway successfull
            if modeLORA == 0:
                print(">LORAWAN_CMD: Sending data by LoRa (SLOW).")
            elif modeLORA == 1:
                print(">LORAWAN_CMD: Sending data by LoRa (FAST).")

            luz         = li.lux()
            pressao     = pres.pressure()
            temperatura = temp()
            humidade    = humi()
            print(" Luminosity: "      + "\t\t" + "{:.2f}".format(luz)         + "\t lux")
            print(" Atmospheric Pressure: "           + "\t\t" + "{:.2f}".format(pressao/100) + "\t hPa")
            print(" Temperature: "       + "\t\t" + "{:.2f}".format(temperatura) + "\t ºC")
            print(" Relative Humidity: " + "\t"   + "{:.2f}".format(humidade)    + "\t %RH")

            for i in range(0, 255, 1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(i << 16 | a << 8 | a)
                pycom.rgbled(int(y))
            for i in range(255, 0, -1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(i << 16 | a << 8 | a)
                pycom.rgbled(int(y))

            # adjust to Cayenne LPP format
            luz            = int( round_up( luz ) )
            pressao        = int( round_up( pressao*10/100, 1 ) )                # /100 -> hPa and *10 correct to LPP format
            temperatura    = int( round_up( temperatura*10, 1 ) )                # *10 correct to LPP format
            humidade       = int( round_up( humidade*514, 1 ) ) + 2**32          # 2**32 -> int to uint and *514 correct to LPP format
            # convert to byte array
            ba_luz         = bytearray(struct.pack('>H', luz))
            ba_pressao     = bytearray(struct.pack('>H', pressao))
            ba_temperatura = bytearray(struct.pack('>H', temperatura))
            ba_humidade    = bytearray(struct.pack('>H', humidade))
            lumLPP    = bytearray([0x04, 0x65, ba_luz[0], ba_luz[1]])                  # 1 - Channel | 2- Type | 3 e/ou 4 - Data
            presLPP   = bytearray([0x06, 0x73, ba_pressao[0], ba_pressao[1]])
            tempLPP   = bytearray([0x00, 0x67, ba_temperatura[0], ba_temperatura[1]])
            humLPP    = bytearray([0x02, 0x68, ba_humidade[0]])
            buttonLPP = bytearray([0x0A, 0x01, 0x01])                                  # activate to create button in Cayenne Dashboard App

            # UPLINK
            ba_combine = lumLPP + presLPP + tempLPP + humLPP #+ buttonLPP
            lora_socket.send(ba_combine)
            time.sleep(0.5)

    # TX - MQTT PUBLISH
    cT_TX_MQTT = int(time.time() * 1000)
    if ((cT_TX_MQTT - pT_TX_MQTT) >= int_TX_MQTT):
        pT_TX_MQTT = cT_TX_MQTT

        # PREPARE VARS FOR MQTT PUBLISH
        if doMQTT == 1 and isWifiOk == 1: # if LoRAWAN Uplink failed
            if modeMQTT == 0:
                print(">MQTT_CMD: Sending data by MQTT (SLOW).")
            elif modeMQTT == 1:
                print(">MQTT_CMD: Sending data by MQTT (FAST).")

            luz         = li.lux()
            pressao     = pres.pressure()
            temperatura = temp()
            humidade    = humi()
            print(" Luminosity: "      + "\t\t" + "{:.2f}".format(luz)         + "\t lux")
            print(" Atmospheric Pressure: "           + "\t\t" + "{:.2f}".format(pressao/100) + "\t hPa")
            print(" Temperature: "       + "\t\t" + "{:.2f}".format(temperatura) + "\t ºC")
            print(" Relative Humidity: " + "\t"   + "{:.2f}".format(humidade)    + "\t %RH")

            for i in range(0, 255, 1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(a << 16 | a << 8 | i)
                pycom.rgbled(int(y))
            for i in range(255, 0, -1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(a << 16 | a << 8 | i)
                pycom.rgbled(int(y))

            mqtt_luz   = "luz,lux="   + str(luz)
            mqtt_press = "press,hPa=" + str(pressao/100)
            mqtt_temp  = "temp,C="    + str(temperatura)
            mqtt_humi  = "humi,%RH="  + str(humidade)

            # TOPIC PUBLISH
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/data/1", msg=mqtt_luz)
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/data/2", msg=mqtt_press)
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/data/3", msg=mqtt_temp)
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/data/4", msg=mqtt_humi)
            doMQTT = 0 # reset flags to try LoRa next time

        elif doMQTT == 1 and isWifiOk == 0: # LoRa has failed and WiFi Network not connected
            print("\n>SYSTEM_CMD: Sending data by MQTT failed! There is no connection to a WiFi network!")

    # RX - BLE RECEIVE
    cT_RX_BLE = int(time.time() * 1000)
    if ((cT_RX_BLE - pT_RX_BLE) >= int_RX_BLE):
        pT_RX_BLE = cT_RX_BLE

        bt.start_scan(1)                                                         # scan devices for 1s
        while bt.isscanning():
            adv = bt.get_adv()
            if adv:
                if bt.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL) == 'ADVERTISING_DEVICE_ID': # device is XYZ
                    print("\n>BLE_ADV: Advertisement from the device " + bt.resolve_adv_data(adv.data, Bluetooth.ADV_NAME_CMPL) + " detected!")
                    mfg_data = bt.resolve_adv_data(adv.data, Bluetooth.ADV_T16SRV_CMPL) # Get service data
                    if mfg_data:
                        #print(ubinascii.hexlify(mfg_data))                      # uncomment for debug advertisement data
                        chunks, chunk_size = len(mfg_data), int(len(mfg_data)/4)
                        split = [ mfg_data[i:i+chunk_size] for i in range(0, chunks, chunk_size) ]
                        # First time commands
                        if split[1] == b'\xbb' and split[3] == b'\x01' and last_state == 0: #16-UUID SERVICE 0x2bbb and (0x0000 or 0x0001) # cmd for RUN Mode ON - LED BLINK
                            print(">BLE_CMD: Blinks LED \t\t\t\t\t\t-> BLINK GREEN")
                            print(">BLE_CMD: Turns off the sleeping mode and turns on the run mode \t\t\t-> RUN MODE ON")
                            last_state = 1
                            break
                        elif split[1] == b'\xbb' and split[3] == b'\x00' and last_state == 1: # cmd for SLEEP Mode ON - LED OFF
                            print(">BLE_CMD: Turns off LED \t\t\t\t\t\t-> OFF")
                            print(">BLE_CMD: Turns off the run mode and turns on the sleeping mode \t\t\t-> SLEEP MODE ON")
                            pycom.rgbled(0)
                            last_state = 0
                            break
                        # Retain state commands
                        elif (split[1] == b'\xbb' and split[3] == b'\x01' and last_state == 1): # retain state LED BLINK
                            print(">BLE_CMD: State of the LED remains the same \t\t\t-> KEEP BLINK GREEN")
                            print(">BLE_CMD: Keeps run mode \t\t\t\t\t-> KEEP RUN MODE")
                            break
                        elif(split[1] == b'\xbb' and split[3] == b'\x00' and last_state == 0):  # retain state LED OFF
                            print(">BLE_CMD: State of the LED remains the same \t\t\t-> KEEP OFF")
                            print(">BLE_CMD: Keeps sleeping mode \t\t\t\t\t-> KEEP SLEEP MODE")
                            break
                        else:
                            break

    # RX - MQTT SUBSCRIBE CHECK FOR TOPIC PUBLISH
    cT_RX_MQTT = int(time.time() * 1000)
    if ((cT_RX_MQTT - pT_RX_MQTT) >= int_RX_MQTT):
        pT_RX_MQTT = cT_RX_MQTT

        if wlan.isconnected():
            client.check_msg()                                                   # Check if message has been posted on topic
            time.sleep(1)
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/data/0", msg=str(modeMQTT)) # Cayenne modeMQTT status
            response = "ok," + str(cayenneActuator)
            client.publish(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/response", msg=response)    # feedback to Cayenne for button update

    # BLINK LED GREEN - RUN MODE or LED OFF - SLEEP MODE
    if last_state == 1:
        cT_LED = int(time.time() * 1000)
        if ((cT_LED - pT_LED) >= int_LED):
            pT_LED = cT_LED
            for i in range(0, 255, 1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(a << 16 | i << 8 | a)
                pycom.rgbled(int(y))
            for i in range(255, 0, -1):
                time.sleep_ms(1)
                a = 0x00
                y = hex(a << 16 | i << 8 | a)
                pycom.rgbled(int(y))

    # WiFi re-check network connection (keep-alive)
    cT_WiFi = int(time.time() * 1000)
    if ((cT_WiFi - pT_WiFi) >= int_WiFi):
        pT_WiFi = cT_WiFi

        if not wlan.isconnected():
            wlan.connect(ssid, auth=(WLAN.WPA2, password))
            join_wait = 0
            print('\n>SYSTEM_CMD: Trying new connection to the WiFi network:')
            print('>', end="")
            while True:
                time.sleep(0.25)
                if not wlan.isconnected():
                    print('.', end="")
                    join_wait += 1
                    if join_wait == 50:
                        print("\n>SYSTEM_CMD: Connection to the WiFi network", ssid , ",failed!")
                        isWifiOk = 0
                        join_wait = 0
                        break
                else:
                    print("\n>SYSTEM_CMD: Connection to the WiFi network", ssid, ", success!")
                    print(">WIFI_CMD: Wifi network data.")
                    print(" Device IP: \t", wlan.ifconfig()[0])
                    print(" Network mask: \t", wlan.ifconfig()[1])
                    print(" Gateway IP: \t"  , wlan.ifconfig()[2])
                    print(" DNS Server: \t", wlan.ifconfig()[3])

                    # MQTT BROKER CONFIG
                    broker = "mqtt.mydevices.com"
                    client = MQTTClient("MQTT_CODE_BROKER", broker, user="MQTT_CODE_USER", password="MQTT_CODE_PASSWORD", port=1883)
                    client.set_callback(subscribe_mqtt_cb)
                    client.connect()
                    client.subscribe(topic="v1/MQTT_CODE_USER/things/MQTT_CODE_BROKER/cmd/0") # cayenne button
                    isWifiOk = 1
                    join_wait = 0
                    break
        else:
            print("\n>SYSTEM_CMD: Remains connected to the WiFi network.", ssid, "!")

    # DEEP SLEEP
    if last_state == 0:
        print(">SYSTEM_CMD: Putting LoPy to sleep \t\t\t\t\t-> DEEP SLEEP = 1m30s\n")
        machine.deepsleep(tSLEEP)
