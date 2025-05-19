import time
import bts7960
import servo
import wifi
import machine
from umqtt.simple import MQTTClient
from umqtt.simple import MQTTException
from utime import sleep_ms

DEBUGLOCALBIT = 1
DEBUGMQTTBIT = 2
DEBUGALLCHANNELS = DEBUGLOCALBIT | DEBUGMQTTBIT
DEBUGINPUT = 0
DEBUGOUTPUT = 0
DEBUGMOTOT = 0
DEBUGMOTOR = DEBUGMQTTBIT
DEBUGSERVO = 0
DEBUGMQTT = DEBUGLOCALBIT

motor1 = None
motor2 = None

# Network credentials
WIFISSID = 'RoverNet'
WIFIPWD = 'Opportunity'

# MQTT credentials
MQTT_HOST = "192.168.8.8"
MQTT_USER = "motor1"
MQTT_PWD = "Opportunity"
MQTT_CHANNEL_ROOT = "rover/motor/"
MQTT_CHANNEL_DEBUG = "rover/debug/" + MQTT_USER
MQTT_CHANNEL_REPORT = "rover/report/" + MQTT_USER
MQTT_CHANNEL_CMD = MQTT_CHANNEL_ROOT + "cmd/" + MQTT_USER + "/"
MQTT_CHANNEL_ERROR = MQTT_CHANNEL_ROOT + "error/" + MQTT_USER
MQTT_CHANNEL_HEARTBEAT = MQTT_CHANNEL_ROOT + "heartbeat"
# Motors are stopped afer 5 heartbeat periods without hearing from anybody
MQTT_HEARTBEAT = 2 * 1000 # ms
MQTT_REPORT = 200 # ms between distance reports
heartbeat_time_sent = time.ticks_ms()
heartbeat_time_heard = time.ticks_ms()

# Motor 1 pins
F_EN1 = 19
FPWM1 = 21
BPWM1 = 20
F_IS1 = 27
B_EN1 = F_EN1
B_IS1 = F_IS1

# Motor 2 pins
F_EN2 = 11
FPWM2 = 12
BPWM2 = 13
F_IS2 = 26
B_EN2 = F_EN2
B_IS2 = F_IS2

# Servo pins
SERVO1 = 2
SERVO2 = 3
#Typical ranges are 90 or 180. Sometimes 270.
# SG92R = 180
# DSS-M15S = 270
SERVORANGE = 270

# Encoder pins need to specific GPIO number, not pin
ENC1A = 10 # pin 14
ENC2A = 18 # pin 24

# Wheel direction tracking
wheeldir1 = 0
wheeldir2 = 0
prevdir1 = wheeldir1
prevdir2 = wheeldir2

led = machine.Pin("LED", machine.Pin.OUT)
led.on()

def cmd_speedl(speed):
    global motor1, wheeldir1
    motor1.speed = speed
    dbgstr = "Left motor: " + str(speed)
    debug_msg(DEBUGMOTOR, dbgstr)
    if abs(speed) < 1:
        wheeldir1 = 0
    elif speed > 0:
        wheeldir1 = 1
    else:
        wheeldir1 = -1


def cmd_speedr(speed):
    global motor2, wheeldir2
    motor2.speed = speed
    dbgstr = "Right motor: " + str(speed)
    debug_msg(DEBUGMOTOR, dbgstr)
    if speed == 0:
        wheeldir2 = 0
    elif speed > 0:
        wheeldir2 = 1
    else:
        wheeldir2 = -1


def cmd_anglel(angle):
    global servo1
    servo1.angle = angle
    dbgstr = "Left angle: " + str(angle)
    debug_msg(DEBUGMOTOR, dbgstr)


def cmd_angler(angle):
    global servo2
    servo2.angle = angle
    dbgstr = "Right angle: " + str(angle)
    debug_msg(DEBUGMOTOR, dbgstr)


def mqtt_message(topic, msg):
    global heartbeat_time_heard
    heartbeat_time_heard = time.ticks_ms() # Any message is as good as a heartbeat
    stopic = topic.decode("UTF-8")
    smsg = msg.decode("UTF-8")
    debug_msg(DEBUGINPUT,stopic + ": " + smsg)
    cmdlen = len(MQTT_CHANNEL_CMD)
    if stopic[:cmdlen] == MQTT_CHANNEL_CMD:
        led.on()
        stopic=stopic[cmdlen:]
        if stopic == "speedl":
            cmd_speedl(100 * float(msg))
        elif stopic == "anglel":
            cmd_anglel(float(msg))
        elif stopic == "speedr":
            cmd_speedr(100 * float(msg))
        elif stopic == "angler":
            cmd_angler(float(msg))
        else:
            mqtt_client.publish(MQTT_CHANNEL_ERROR, "Unrecognised command = " + stopic + ":" + smsg)
    elif stopic[:len(MQTT_CHANNEL_HEARTBEAT)] == MQTT_CHANNEL_HEARTBEAT:
        debug_msg(DEBUGINPUT, "Heard heartbeat from " + smsg)
    else:
        debug_msg(DEBUGINPUT,"Not a command: " + stopic + ": " + smsg)
    led.off()


def wifi_connect():
    if not wificlient.isconnected():
        #print("Connecting to WiFi")
        wificlient.connect()


def debug_msg(flag,msg):
    global mqtt_connected
    if mqtt_connected and ((flag & DEBUGMQTTBIT) != 0):
        mqtt_client.publish(MQTT_CHANNEL_DEBUG, msg)
    if (flag & DEBUGLOCALBIT) != 0:
        print(msg)


def blink(n):
    #print ("Blink:",n)
    for i in range(n):
        led.on()
        time.sleep(0.05)
        led.off()
        time.sleep(0.05)
    time.sleep(0.5)


def mqtt_connect():
    global heartbeat_time_sent
    global heartbeat_time_heard
    try:
        if mqtt_client is not None:
            mqtt_client.disconnect()
            time.sleep(5)
    except:
        pass
    mqtt_client = MQTTClient(
            client_id=MQTT_USER,
            server=MQTT_HOST,
            user=MQTT_USER,
            password=MQTT_PWD)
    #print(mqtt_client.__dict__)
    mqtt_connected = False
    errcount = 0
    while not mqtt_connected:
        try:
            if errcount > 3:
                #print("Too many connection attempts, trying reboot")
                time.sleep(0.2)
                machine.reset()
            wifi_connect()
            time.sleep(1)
            #print("Connecting to MQTT", errcount)
            mqtt_client.set_callback(mqtt_message)
            mqtt_client.connect()
            mqtt_connected = True
        except Exception as e:
            print(e)
            errcount += 1
    #debug_msg(DEBUGMQTT,"Listening on " + MQTT_CHANNEL_CMD + "#")
    mqtt_client.subscribe(MQTT_CHANNEL_CMD + "#")
    #debug_msg(DEBUGMQTT,"Listening on " + MQTT_CHANNEL_ROOT + "join")
    mqtt_client.subscribe(MQTT_CHANNEL_ROOT + "join")
    #debug_msg(DEBUGMQTT,"Listening on " + MQTT_CHANNEL_HEARTBEAT)
    mqtt_client.subscribe(MQTT_CHANNEL_HEARTBEAT)
    mqtt_client.publish(MQTT_CHANNEL_ROOT+"join", "Hello from " + MQTT_USER)
    heartbeat_time_sent = time.ticks_ms()
    heartbeat_time_heard = time.ticks_ms()
    blink(10)
    return mqtt_client
    

def network_connect():
    # Kill motors
    cmd_speedl(0.0)
    cmd_speedr(0.0)
    blink(3)
    wifi_connect();
    blink(6)
    #print("Connected to WiFi")
    return mqtt_connect();


def send_heartbeat():
    global heartbeat_time_sent
    global led
    debug_msg(DEBUGOUTPUT, "Sending heartbeat")
    mqtt_client.publish(MQTT_CHANNEL_HEARTBEAT, MQTT_USER)
    heartbeat_time_sent = now
    led.on()
    time.sleep(0.02)
    led.off()


def mqttreport():
    global wheeldir1, wheeldir2
    global prevdir1, prevdir2
    global pulsecount1, pulsecount2
    global servo1, servo2
    msg = str(pulsecount1 * wheeldir1)+"@"+str(servo1.angle)
    mqtt_client.publish(MQTT_CHANNEL_REPORT+"/left", msg)
    pulsecount1 = 0
    msg = str(pulsecount2 * wheeldir2)+"@"+str(servo2.angle)
    mqtt_client.publish(MQTT_CHANNEL_REPORT+"/right", msg)
    pulsecount2 = 0
    

def checkDirectionChange():
    global wheeldir1, wheeldir2
    global prevdir1, prevdir2
    if wheeldir1 != prevdir1 or wheeldir2 != prevdir2:
        mqttreport()
    prevdir1 = wheeldir1
    prevdir2 = wheeldir2

def enc1Acallback(pin):
    #print("1A")
    global pulsecount1
    pulsecount1 += 1


def enc2Acallback(pin):
    #print("2A")
    global pulsecount2
    pulsecount2 += 1
    

motor1 = bts7960.motor((F_EN1, FPWM1), (B_EN1, BPWM1))
motor2 = bts7960.motor((F_EN2, FPWM2), (B_EN2, BPWM2))
# Timings for SG92R = 575..2425
# Timings for DSS-M15S = 500..2500
servo1 = servo.Servo(SERVO1, SERVORANGE, 500, 2500, 50)
servo2 = servo.Servo(SERVO2, SERVORANGE, 500, 2500, 50)

enc1Apin = machine.Pin(ENC1A, machine.Pin.IN, machine.Pin.PULL_DOWN)
enc2Apin = machine.Pin(ENC2A, machine.Pin.IN, machine.Pin.PULL_DOWN)
pulsecount1 = 0
pulsecount2 = 0
enc1Apin.irq(trigger=machine.Pin.IRQ_RISING, handler = enc1Acallback)
enc2Apin.irq(trigger=machine.Pin.IRQ_RISING, handler = enc2Acallback)

wificlient = wifi.WIFI(WIFISSID, WIFIPWD)
mqtt_connected = False
led.off()
loopcount = 0
reporttime = time.ticks_ms()
while True:
    try:
        if not wificlient.isconnected() or not mqtt_connected:
            mqtt_client = network_connect()
        mqtt_connected = True
        mqtt_client.check_msg()
        now = time.ticks_ms()
        if time.ticks_diff(now, heartbeat_time_sent) > MQTT_HEARTBEAT:
            send_heartbeat()
            #print(pulsecount1, pulsecount2)
        wait_time = time.ticks_diff(now, heartbeat_time_heard)
        if wait_time > MQTT_HEARTBEAT * 2:
            debug_msg(DEBUGMQTT, "It's quiet.")
            if wait_time > MQTT_HEARTBEAT * 5:
                debug_msg(DEBUGMQTT, "    Too quiet!")
                # Kill motors
                cmd_speedl(0.0)
                cmd_speedr(0.0)
        if time.ticks_diff(now, reporttime) > MQTT_REPORT:
            mqttreport()
            # print(loopcount, wheeldir1, prevdir1, pulsecount1, wheeldir2, prevdir2, pulsecount2)
            reporttime = now
            loopcount = 0
        checkDirectionChange()
        loopcount += 1
    except MQTTException as e:
        debug_msg(DEBUGMQTT, "MQTT exception raised: ", e)
        mqtt_connected = False