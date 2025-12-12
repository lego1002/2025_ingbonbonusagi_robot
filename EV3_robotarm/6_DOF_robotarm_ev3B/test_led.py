#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import os
import time

# ======== Device Config ========
DEVICE_NAME = "ev3B"
BROKER = "10.89.89.64"   # Your computer (MQTT broker)
PORT   = 1883
USER   = "lego"
PASS   = "lego"

# ======== Topic Config ========
TOPIC_CMD    = DEVICE_NAME + "/led/light"
TOPIC_STATUS = DEVICE_NAME + "/led/status"

# ======== LED Paths ========
LED_PATHS = [
    ("led0:red:brick-status",   "red"),
    ("led0:green:brick-status", "green"),
    ("led1:red:brick-status",   "red"),
    ("led1:green:brick-status", "green"),
]

# ---------- Low-latency LED write ----------
def write_brightness(path, value):
    try:
        with open(path, "w") as f:
            f.write(value)
        return True
    except Exception:
        try:
            os.system('echo {} | sudo tee {} > /dev/null'.format(value, path))
            return True
        except Exception:
            return False

def set_led(color):
    for name, c in LED_PATHS:
        path = "/sys/class/leds/{}/brightness".format(name)
        if color == "off":
            val = "0"
        elif color == "amber":
            val = "255"
        elif color == c:
            val = "255"
        else:
            val = "0"
        ok = write_brightness(path, val)
        if not ok:
            print("Write failed: {} -> {}".format(path, val))
    print("[{}] LED -> {}".format(DEVICE_NAME, color))

# ---------- MQTT Callbacks ----------
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[{}] Connected to MQTT broker, subscribing to {}".format(DEVICE_NAME, TOPIC_CMD))
        client.subscribe(TOPIC_CMD, qos=0)
        client.publish(TOPIC_STATUS, "{} online".format(DEVICE_NAME), qos=0, retain=False)
    else:
        print("[{}] MQTT connection failed, rc={}".format(DEVICE_NAME, rc))

def on_disconnect(client, userdata, rc):
    print("[{}] MQTT disconnected, rc={}".format(DEVICE_NAME, rc))

def on_message(client, userdata, msg):
    payload = msg.payload.decode().strip().lower()
    print("[{}] Received command: {}".format(DEVICE_NAME, payload))
    set_led(payload)
    client.publish(TOPIC_STATUS, "LED set to {}".format(payload), qos=0, retain=False)

# ---------- Main ----------
def main():
    client = mqtt.Client(client_id=DEVICE_NAME, clean_session=True)
    if USER:
        client.username_pw_set(USER, PASS)

    try:
        client.reconnect_delay_set(min_delay=1, max_delay=5)
    except Exception:
        pass

    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    print("[{}] Connected to MQTT {}:{}, waiting for commands...".format(DEVICE_NAME, BROKER, PORT))
    print("Command topic: {} | Status topic: {}".format(TOPIC_CMD, TOPIC_STATUS))

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
