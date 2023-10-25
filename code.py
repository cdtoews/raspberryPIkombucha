import os
import microcontroller
import ssl
import socketpool
import ipaddress
import wifi
import time
import board
import adafruit_requests
import digitalio
import busio
import adafruit_minimqtt.adafruit_minimqtt as MQTT
import microcontroller
import rtc
import adafruit_ntp


# ------------- WATCHDOG ---------------------
isWatchdog = DEBUG = (os.getenv('ISWATCHDOG', 'False') == 'True')

if isWatchdog:
    print(time.monotonic(), "-- watchdog is active")
    import watchdog
else:
    print(time.monotonic(), "-- watchdog is NOT active")

#-------------- SENSOR TYPE -------------------------
sensorType = os.getenv('SENSORTYPE')



if sensorType == "AHT20":
    # ---------- AHT20 on GP0 and GP1 -------------
    print(time.monotonic(), " -- Sensor Type = AHT20")
    import adafruit_ahtx0
    i2c = busio.I2C(board.GP1, board.GP0)
    aht20 = adafruit_ahtx0.AHTx0(i2c)

    def getTempF():
        global aht20
        temperature = aht20.temperature
        return float( 9.0/5.0 * temperature + 32)

    def getHumidity():
        return  aht20.relative_humidity
    # ----------  END of AHT20 --------------
elif sensorType == "HTU31D":
    # ---------- HTU31D on GP0 and GP1 -------------
    print(time.monotonic(), " -- Sensor Type = HTU31D")
    import adafruit_htu31d
    print(time.monotonic(), " --starting HTU31D")
    i2c = busio.I2C(board.GP1, board.GP0)
    htu = adafruit_htu31d.HTU31D(i2c)
    print(time.monotonic(), " --Found HTU31D with serial number", hex(htu.serial_number))
    
    def getTempF():
        temperature = htu.temperature
        tempF = 9.0/5.0 * temperature + 32
        return tempF
    
    def getHumidity():
        relative_humidity = htu.relative_humidity
        return relative_humidity
    # ----------  END of HTU31D --------------
elif sensorType == "SHT40":
    # ---------- SHT40 on GP0 and GP1 -------------
    print(time.monotonic(), " -- Sensor Type = SHT40")
    import adafruit_sht4x
    print(time.monotonic(), " --starting SHT40")
    i2c = busio.I2C(board.GP1, board.GP0)
    sht = adafruit_sht4x.SHT4x(i2c, address=0x44)
    print(time.monotonic(), " --Found SHT4x with serial number", hex(sht.serial_number))

    sht.mode = adafruit_sht4x.Mode.NOHEAT_HIGHPRECISION
    # Can also set the mode to enable heater
    # sht.mode = adafruit_sht4x.Mode.LOWHEAT_100MS
    print(time.monotonic(), " --Current mode is: ", adafruit_sht4x.Mode.string[sht.mode])
    def getTempF():
        temperature = sht.temperature
        tempF = 9.0/5.0 * temperature + 32
        return tempF
    
    def getHumidity():
        relative_humidity = sht.relative_humidity
        return relative_humidity
    # ----------  END of SHT40 --------------
else:
    print(time.monotonic(), "###### We didn't get a sensor Type...")

#keep track of what day we sync'd RTC to NTP
currentDOY = -1

# how long to wait between sensor data sends
MQTT_SEND_TIMER = 60
MQTT_LAST_SEND_TIME = -60

# set default values until we get upates
HIGH_TEMP = float(79)
LOW_TEMP = float(77)
MEDIUM_PERCENTAGE = 50
MEDIUM_LOOP_TOTAL_SEC = 300 #300 = 5 minutes
MEDIUM_LOOP_ON_SEC = MEDIUM_LOOP_TOTAL_SEC * (MEDIUM_PERCENTAGE / 100)
CHECK_TEMP_EVERY_SEC = 5 #check temps every 5 seconds
LAST_TEMP_CHECK = -60
HEATER_STATUS = 0
THERMOSTAT_STATUS = 0
STATUS_OFF = 0
STATUS_MEDIUM = 1
STATUS_HIGH = 2


mqtt_server = os.getenv('MQTT_SERVER')
mqtt_user = os.getenv('MQTT_USER')
mqtt_pass = os.getenv('MQTT_PASS')
mqtt_client_id = os.getenv('MQTT_CLIENT_ID')

print(time.monotonic(), " --My CLIENT ID is: " , mqtt_client_id)

topic_temp = mqtt_client_id + os.getenv('MQTT_TEMP')
topic_humidity = mqtt_client_id +  os.getenv('MQTT_HUMIDITY')
topic_thermostat = mqtt_client_id +  os.getenv('MQTT_THERMO_STATUS')
topic_medium_percentage = mqtt_client_id + os.getenv('MQTT_MEDIUM_PERCENTAGE_STATUS')
topic_heater_status = mqtt_client_id +  os.getenv('MQTT_HEATER_STATUS')
topic_update = mqtt_client_id +  os.getenv('MQTT_UPDATE_SETTINGS')
topic_connect_msg = 'connected'

#-------FEEDS- SUBSCRIBED TO--------------
onboardled_feed = mqtt_client_id +  os.getenv('MQTT_ONBOARD_LED_FEED')
hightemp_feed = mqtt_client_id +  os.getenv('MQTT_HIGHTEMP_FEED')
lowtemp_feed = mqtt_client_id +  os.getenv('MQTT_LOWTEMP_FEED')
medium_percentage_feed = mqtt_client_id +  os.getenv('MQTT_MEDIUMPERCENTAGE_FEED')

LED = digitalio.DigitalInOut(board.LED)
LED.direction = digitalio.Direction.OUTPUT

LED1 = digitalio.DigitalInOut(board.GP5) #default is GP15
LED1.direction = digitalio.Direction.OUTPUT

RELAY1 = digitalio.DigitalInOut(board.GP15)
RELAY1.direction = digitalio.Direction.OUTPUT

RELAYindicator = digitalio.DigitalInOut(board.GP28)#default is GP14
RELAYindicator.direction = digitalio.Direction.OUTPUT

# --------------------- functions ------------------------

#timestamp function
def getTimestamp():
    
    # currentDOY
    try:
        global currentDOY, rtc, ntp
        if currentDOY != time.localtime().tm_yday:
            #once a day, sync
            print(time.monotonic(), " -- Syncing RTC to NTP")
            rtc.RTC().datetime = ntp.datetime
            currentDOY = time.localtime().tm_yday
        return "{0}/{1:0>2}/{2:0>2}T{3:0>2}:{4:0>2}:{5:0>2} -".format(time.localtime().tm_year ,
                                               time.localtime().tm_mon,
                                               time.localtime().tm_mday,
                                               time.localtime().tm_hour,
                                               time.localtime().tm_min,
                                               time.localtime().tm_sec)
    except Exception as e:
        return time.monotonic()
      



def configure_medium():
    global MEDIUM_LOOP_ON_SEC, MEDIUM_LOOP_TOTAL_SEC, MEDIUM_PERCENTAGE
    MEDIUM_LOOP_ON_SEC = MEDIUM_LOOP_TOTAL_SEC * (MEDIUM_PERCENTAGE / 100) # how long for heater to be on
    print(getTimestamp(), " --total medium loop: {0}, heater on for: {1}, percentage: {2}".format(MEDIUM_LOOP_TOTAL_SEC, MEDIUM_LOOP_ON_SEC, MEDIUM_PERCENTAGE))

def connected(client, userdata, flags, rc):
    # This function will be called when the client is connected
    # successfully to the broker.
    print(getTimestamp(), " --Connected to MQTT SERVER! Listening for topic changes")
    # Subscribe to all changes on the onoff_feed.
    client.subscribe(onboardled_feed)
    client.subscribe(hightemp_feed)
    client.subscribe(lowtemp_feed)
    client.subscribe(medium_percentage_feed)
    


def disconnected(client, userdata, rc):
    # This method is called when the client is disconnected
    print(getTimestamp(), " --Disconnected from MQTT")
    reconnect()

def message(client, topic, message):
    global HIGH_TEMP, LOW_TEMP, MEDIUM_PERCENTAGE
    # This method is called when a topic the client is subscribed to
    # has a new message.
    led_on()
    print(getTimestamp(), " --New message on topic {0}: {1}".format(topic, message))
    if topic == onboardled_feed:
        if message == "1":
            LED.value = True
        elif message == "0":
            LED.value = False
    elif topic == hightemp_feed:
        HIGH_TEMP = float(message)
        print(getTimestamp(), " high temp set to {0}".format(message))
    elif topic ==  lowtemp_feed:
        LOW_TEMP = float(message)
        print(getTimestamp(), " low temp set to {0}".format(message))
    elif topic == medium_percentage_feed:
        MEDIUM_PERCENTAGE = float(message)
        configure_medium()
        print(getTimestamp(), " medium percentage set to {0}".format(MEDIUM_PERCENTAGE))


    led_off()
   
def reconnect():
    print(getTimestamp(), " --\n\nTRYING TO RECONNECT TO MQTT\n\n")
    connect_count = 0
    while True:
        try:
            mqtt_client.connect()
            print(getTimestamp(), " --RECONNECTED TO MQTT SUCCESSFULLY")
            return
        except Exception as e:
            print(getTimestamp(), " --trouble connecting, pausing for 30 seconds\n", e)
            connect_count += 1
            if (connect_count > 5):
                print(getTimestamp(), " --tried a few times to reconnect, rebooting now")
                microcontroller.reset()
            time.sleep(30)
            
            
      
def forceUpdate():
    global MQTT_LAST_SEND_TIME
    MQTT_LAST_SEND_TIME= -60

def led_on():
    LED1.value = True
    
def led_off():
    LED1.value = False
    
def relay_on():
    global HEATER_STATUS
    RELAY1.value = True
    RELAYindicator.value = True
    HEATER_STATUS = 1
    
def relay_off():
    global HEATER_STATUS
    RELAY1.value = False
    RELAYindicator.value = False
    HEATER_STATUS = 0

# ------------- end of functions ---------------------
configure_medium()

print(getTimestamp(), " --Connecting to WiFi")

#  connect to your SSID
wifi.radio.connect(os.getenv('SSID'), os.getenv('WIFIPASS'))

print(getTimestamp(), " --Connected to WiFi")

pool = socketpool.SocketPool(wifi.radio)

#  prints MAC address to REPL
print(getTimestamp(), " --My MAC addr:", [hex(i) for i in wifi.radio.mac_address])

#  prints IP address to REPL
print(getTimestamp(), " --My IP address is", wifi.radio.ipv4_address)

ping_ip = ipaddress.IPv4Address("8.8.8.8")
ping = wifi.radio.ping(ip=ping_ip)

# retry once if timed out
if ping is None:
    ping = wifi.radio.ping(ip=ping_ip)

if ping is None:
    print(getTimestamp(), "Couldn't ping 'google.com' successfully")
else:
    # convert s to ms
    print(getTimestamp(), f"Pinging 'google.com' took: {ping * 1000} ms")

# get NTP time
ntp = adafruit_ntp.NTP(pool, tz_offset=0)
rtc.RTC().datetime = ntp.datetime


try:
    led_on()
    print(getTimestamp(), " --Connecting to MQTT Server")
    # Set up a MiniMQTT Client
    mqtt_client = MQTT.MQTT(
        broker=mqtt_server,
        username=mqtt_user,
        password=mqtt_pass,
        socket_pool=pool,
        ssl_context=ssl.create_default_context(),
       )
    # Setup the callback methods above
    mqtt_client.on_connect = connected
    mqtt_client.on_disconnect = disconnected
    mqtt_client.on_message = message
    mqtt_client.connect()
    #send update request
    mqtt_client.publish(topic_update, 1)
    
    led_off()
except Exception as e:
    print(getTimestamp(), " --errors connecting to MQTT, resetting\n",e)
    microcontroller.reset()
    led_off()

if isWatchdog:
    #turn on watchdog
    wdt = microcontroller.watchdog
    wdt.timeout = 8
    wdt.mode = watchdog.WatchDogMode.RESET

while True:
    #feed the watchdog
    if isWatchdog:
        wdt.feed()
    
    try:
        mqtt_client.loop()
    except Exception as e:
        print(getTimestamp(), " --Failed to connect to MQTT server\n", e)
        reconnect()
    # catch then supervisor.reload()
    now = time.monotonic()
    if now >= MQTT_LAST_SEND_TIME + MQTT_SEND_TIMER:
        #if it's time to send an MQTT update
        led_on()
        MQTT_LAST_SEND_TIME = now
        #print(time.monotonic(), "#############################3type of aht20.temperature: {0}".format(type(aht20.temperature)))
        
        TempF = getTempF()
        relative_humidity = getHumidity()
        #relative_humidity = aht20.relative_humidity
                
        print(getTimestamp(), " --Temperature: %0.1f F" % TempF)
        print(getTimestamp(), " --Humidity: %0.1f " % relative_humidity)
        print("")
        # Send a new message
        #print("Sending temmp value: %d..." % TempF)
        mqtt_client.publish(topic_temp, round(TempF, 1))
        mqtt_client.publish(topic_humidity, round(relative_humidity, 1))
        mqtt_client.publish(topic_thermostat, THERMOSTAT_STATUS)
        mqtt_client.publish(topic_medium_percentage, MEDIUM_PERCENTAGE)
        mqtt_client.publish(topic_heater_status, HEATER_STATUS)
        
        
        
        
        print(getTimestamp(), " --MQTT DATA Sent!")
        led_off()
    elif (now > (LAST_TEMP_CHECK + CHECK_TEMP_EVERY_SEC)):
        #time to check temp
        led_on()
        
        TempF = getTempF()
        if (TempF > HIGH_TEMP) and (THERMOSTAT_STATUS != STATUS_OFF):
            #if we are over high temp and not already on off
            THERMOSTAT_STATUS = STATUS_OFF
            relay_off()
            print(getTimestamp(), " --Temp over limit, turning relay off")
            #change medium percentage
            MEDIUM_PERCENTAGE -= 1
            configure_medium()
            forceUpdate()
        elif (TempF <= LOW_TEMP) and (THERMOSTAT_STATUS != STATUS_HIGH):
            #if we are below low temp ad not already at HIGH
            THERMOSTAT_STATUS = STATUS_HIGH
            relay_on()
            print(getTimestamp(), " --Temp under limit, turning relay on")
            #change medium percentage
            MEDIUM_PERCENTAGE += 1
            configure_medium()
            forceUpdate()
        elif (TempF >= LOW_TEMP) and (TempF <= HIGH_TEMP):
            
            #figure out where we are in our medium loop
            current_loop_sec = now % MEDIUM_LOOP_TOTAL_SEC
            if (current_loop_sec <= MEDIUM_LOOP_ON_SEC):
                relay_on()
                #print(time.monotonic(), " --IN MEDIUM, turning relay on")
                
            else:
                relay_off()
                #print(time.monotonic(), " --IN MEDIUM, turning relay off")
                
                
                
            if THERMOSTAT_STATUS != STATUS_MEDIUM:
                #we are just coming into medium
                THERMOSTAT_STATUS = STATUS_MEDIUM
                print(getTimestamp(), " --Temp between high and low, putting to medium")
                forceUpdate()
        #else:
            #print(time.monotonic()," -------- HOW DID WE GET HERE!!!")
            
        
        LAST_TEMP_CHECK = now #reset timer
        led_off()




