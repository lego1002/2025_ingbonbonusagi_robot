#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import time
import glob
import subprocess

def cleanup_tacho_motors():
    for mpath in glob.glob("/sys/class/tacho-motor/motor*"):
        try:
            os.system("echo stop > {}/command 2>/dev/null".format(mpath))
        except:
            pass
        try:
            # 這裡一定要 echo，不然會變成執行 motor5 這種指令
            os.system("echo {} | sudo tee /sys/bus/lego/drivers/lego-motor/unbind > /dev/null 2>&1"
                      .format(os.path.basename(mpath)))
        except:
            pass
    time.sleep(0.5)


def main():
    base_dir = os.path.dirname(os.path.abspath(__file__))
    motorA_py = os.path.join(base_dir, "ev3_motorA.py")
    motorB_py = os.path.join(base_dir, "ev3_motorB.py")
    motorC_py = os.path.join(base_dir, "ev3_motorC.py")

    # === 只要改這裡的 IP，就能改全部 motor 的 broker ===
    BROKER_IP = "10.13.209.64"

    cleanup_tacho_motors()

    procs = []
    try:
        print("[LAUNCH] start motorA...")
        pA = subprocess.Popen(["python3", motorA_py, "--broker", BROKER_IP])
        procs.append(pA)

        time.sleep(0.5)

        print("[LAUNCH] start motorB...")
        pB = subprocess.Popen(["python3", motorB_py, "--broker", BROKER_IP])
        procs.append(pB)

        time.sleep(0.5)

        print("[LAUNCH] start motorC...")
        pC = subprocess.Popen(["python3", motorC_py, "--broker", BROKER_IP])
        procs.append(pC)

        print("[LAUNCH] motors started. Press Ctrl+C to stop.")

        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n[LAUNCH] stopping motors...")
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        time.sleep(1)
        for p in procs:
            if p.poll() is None:
                p.kill()

        print("[LAUNCH] all processes stopped.")


if __name__ == "__main__":
    main()
