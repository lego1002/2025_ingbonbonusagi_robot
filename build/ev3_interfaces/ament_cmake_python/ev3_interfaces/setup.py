from setuptools import find_packages
from setuptools import setup

setup(
    name='ev3_interfaces',
    version='0.0.0',
    packages=find_packages(
        include=('ev3_interfaces', 'ev3_interfaces.*')),
)
