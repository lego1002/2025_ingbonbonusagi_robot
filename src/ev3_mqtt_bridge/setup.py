from setuptools import find_packages, setup

package_name = 'ev3_mqtt_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ros_mqtt_bridge_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jacky',
    maintainer_email='jackylee20030214@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ros_mqtt_bridge = ev3_mqtt_bridge.ros_mqtt_bridge:main',
            'led_controller = ev3_mqtt_bridge.led_controller:main',
            'motor_action = ev3_mqtt_bridge.motor_action:main',
            'motorA_controller = ev3_mqtt_bridge.motorA_controller:main',
            'motorB_controller = ev3_mqtt_bridge.motorB_controller:main',
            'motorC_controller = ev3_mqtt_bridge.motorC_controller:main',
            'motorD_controller = ev3_mqtt_bridge.motorD_controller:main',
            'motorE_controller = ev3_mqtt_bridge.motorE_controller:main',
        ],
    },
)
