import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jacky/Documents/robot_bartender/install/ev3_mqtt_bridge'
