#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import paho.mqtt.client as mqtt
import os
import time
import glob
import re
import os, glob, time
import argparse



# ====== 清理殘留的 motor port ======
def cleanup_tacho_motors():
    for mpath in glob.glob("/sys/class/tacho-motor/motor*"):
        try:
            # 嘗試停止舊 motor
            os.system("echo stop > {}/command 2>/dev/null".format(mpath))
        except:
            pass
        try:
            # 嘗試解除綁定舊 motor 節點
            os.system("echo {} | sudo tee /sys/bus/lego/drivers/lego-motor/unbind > /dev/null 2>&1"
                      .format(os.path.basename(mpath)))
        except:
            pass
    time.sleep(0.5)

# cleanup_tacho_motors()

# ====== 設定 ======
parser = argparse.ArgumentParser()
parser.add_argument("--broker", type=str, default="127.0.0.1",
                    help="MQTT broker IP")
parser.add_argument("--port", type=int, default=1883)
parser.add_argument("--user", type=str, default="lego")
parser.add_argument("--password", type=str, default="lego")
args = parser.parse_args()

BROKER = args.broker
PORT   = args.port
USER   = args.user
PASS   = args.password

# 與電腦端/橋接一致的 topic（EV3A / motorA）
TOPIC_CMD    = "ev3B/motorD/cmd"
TOPIC_STATUS = "ev3B/motorD/status"

# 這台 EV3 的「本馬達 ID」；只有當收到 m{MY_ID}: 前綴或 無前綴(廣播) 才會執行
MY_ID = 4

# 把 motor_id 對應到 EV3 實體輸出口（你要的對應：1 -> outA）
ID_TO_PORT = {
    4: "outA",
    # 之後要加：2:"outB", 3:"outC", 4:"outD" ...
}

# ====== 工具：安全讀寫 sysfs（先 open，失敗再 tee 後備）======
def sys_write(path, value):
    try:
        with open(path, "w") as f:
            f.write(str(value))
        return True
    except Exception:
        try:
            os.system('echo {} | sudo tee {} > /dev/null'.format(value, path))
            return True
        except Exception:
            return False

def sys_read(path):
    try:
        with open(path, "r") as f:
            return f.read().strip()
    except Exception:
        return ""

# ====== 尋找指定輸出口的馬達目錄 ======
def find_motor_by_port(port_name):
    for mpath in glob.glob("/sys/class/tacho-motor/motor*"):
        addr = sys_read(os.path.join(mpath, "address"))
        if port_name in addr:
            return mpath
    return None

# ====== 回報目前狀態 ======
def publish_status(client, motor, note):
    if motor is None:
        client.publish(TOPIC_STATUS, "ERR: motor {} not found".format(ID_TO_PORT.get(MY_ID, "unknown")))
        return
    pos   = sys_read(os.path.join(motor, "position"))
    state = sys_read(os.path.join(motor, "state"))
    duty  = sys_read(os.path.join(motor, "duty_cycle"))
    spd   = sys_read(os.path.join(motor, "speed"))
    msg = "note={}, pos={}, state={}, duty={}, speed={}".format(note, pos, state, duty, spd)
    client.publish(TOPIC_STATUS, msg, qos=0, retain=False)

# ====== 馬達基本操作 ======
def cmd_run_forever(motor):
    sys_write(os.path.join(motor, "command"), "run-forever")

def cmd_stop(motor):
    sys_write(os.path.join(motor, "command"), "stop")

def cmd_reset(motor):
    sys_write(os.path.join(motor, "command"), "reset")

def set_duty(motor, duty):
    sys_write(os.path.join(motor, "duty_cycle_sp"), str(int(duty)))
    # 立即以 duty 方式驅動；若只想預設待命可拿掉下一行
    sys_write(os.path.join(motor, "command"), "run-direct")

def run_to_rel_pos(motor, deg, speed):
    sys_write(os.path.join(motor, "position_sp"), str(int(deg)))
    sys_write(os.path.join(motor, "speed_sp"),    str(int(speed)))
    sys_write(os.path.join(motor, "command"),     "run-to-rel-pos")

def run_to_abs_pos(motor, deg, speed):
    sys_write(os.path.join(motor, "position_sp"), str(int(deg)))
    sys_write(os.path.join(motor, "speed_sp"),    str(int(speed)))
    sys_write(os.path.join(motor, "command"),     "run-to-abs-pos")

# ====== 解析指令：支援可選 m{id}: 前綴 ======
PREFIX_RE = re.compile(r'^\s*m(\d+)\s*:\s*(.*)$', re.IGNORECASE)

def extract_target_and_cmd(payload):
    """
    回傳 (target_id 或 None, cmd_str)
      - 有 m{id}: 前綴 → (id, 剩下的字串)
      - 無前綴 → (None, 原字串)
    """
    m = PREFIX_RE.match(payload)
    if m:
        return int(m.group(1)), m.group(2).strip()
    return None, payload.strip()

# ====== MQTT 回呼 ======
MOTOR_PATH = None
LAST_HB = 0

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        client.subscribe(TOPIC_CMD, qos=0)
        client.publish(TOPIC_STATUS, "EV3 motor node online (id={}, port={})".format(MY_ID, ID_TO_PORT.get(MY_ID, "?")), qos=0, retain=False)
        publish_status(client, MOTOR_PATH, "connected")
    else:
        client.publish(TOPIC_STATUS, "ERR: connect rc={}".format(rc))

def on_message(client, userdata, msg):
    payload = msg.payload.decode('utf-8', 'ignore').strip()
    target_id, cmd = extract_target_and_cmd(payload)
    low = cmd.lower()
    note = "ignored"

    # 只在 1) 無前綴(廣播) 或 2) 前綴 id == MY_ID 時執行
    if (target_id is not None) and (target_id != MY_ID):
        return

    if MOTOR_PATH is None:
        client.publish(TOPIC_STATUS, "ERR: motor {} not found".format(ID_TO_PORT.get(MY_ID, "unknown")))
        return

    try:
        if low == "run":
            cmd_run_forever(MOTOR_PATH)
            note = "run"
        elif low == "stop":
            cmd_stop(MOTOR_PATH)
            note = "stop"
        elif low == "reset":
            cmd_reset(MOTOR_PATH)
            note = "reset"
        elif low.startswith("duty"):
            parts = low.split()
            if len(parts) >= 2:
                val = int(float(parts[1]))
                if val < -100: val = -100
                if val >  100: val = 100
                set_duty(MOTOR_PATH, val)
                note = "duty {}".format(val)
            else:
                note = "ERR: duty <val>"
        elif low.startswith("rel"):
            parts = low.split()
            if len(parts) >= 2:
                deg = int(float(parts[1]))
                spd = 300
                if len(parts) >= 3:
                    spd = int(float(parts[2]))
                run_to_rel_pos(MOTOR_PATH, deg, spd)
                note = "rel {} speed {}".format(deg, spd)
            else:
                note = "ERR: rel <deg> [speed]"
        elif low.startswith("abs"):
            parts = low.split()
            if len(parts) >= 2:
                deg = int(float(parts[1]))
                spd = 300
                if len(parts) >= 3:
                    spd = int(float(parts[2]))
                run_to_abs_pos(MOTOR_PATH, deg, spd)
                note = "abs {} speed {}".format(deg, spd)
            else:
                note = "ERR: abs <deg> [speed]"
        elif low.startswith("home"):
            parts = low.split()
            spd = 300
            if len(parts) >= 2:
                spd = int(float(parts[1]))
            # home = 以 speed_sp = spd 跑到 abs 位置 0
            run_to_abs_pos(MOTOR_PATH, 0, spd)
            note = "home speed {}".format(spd)
        else:
            note = "ERR: unknown cmd '{}'".format(payload)
    except Exception as e:
        note = "ERR: {}".format(e)

    publish_status(client, MOTOR_PATH, note)

# ====== 主程式 ======
def main():
    global MOTOR_PATH, LAST_HB

    port = ID_TO_PORT.get(MY_ID, None)
    if port is None:
        print("Config error: MY_ID={} has no port mapping".format(MY_ID))
        return

    MOTOR_PATH = find_motor_by_port(port)

    # 盡量相容舊版 paho-mqtt
    try:
        client = mqtt.Client(clean_session=True)
    except TypeError:
        client = mqtt.Client()

    if USER:
        client.username_pw_set(USER, PASS)

    try:
        client.reconnect_delay_set(min_delay=1, max_delay=5)
    except Exception:
        pass

    client.on_connect = on_connect
    client.on_message = on_message

    client.connect(BROKER, PORT, keepalive=60)
    client.loop_start()

    print("Broker {}:{}".format(BROKER, PORT))
    print("CMD={} | STATUS={}".format(TOPIC_CMD, TOPIC_STATUS))
    print("MY_ID={} -> port={} | path={}".format(MY_ID, port, MOTOR_PATH if MOTOR_PATH else "NOT FOUND"))

    try:
        while True:
            now = time.time()
            if now - LAST_HB > 5.0:
                publish_status(client, MOTOR_PATH, "heartbeat")
                LAST_HB = now
            time.sleep(0.2)
    except KeyboardInterrupt:
        pass
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()
