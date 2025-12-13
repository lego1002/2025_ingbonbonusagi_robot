from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'flask_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", 
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        
        (os.path.join("share", package_name, "static"),
         glob("flask_ui/static/**/*", recursive=True)),

        (os.path.join("share", package_name, "templates"),
         glob("flask_ui/templates/*")),
    ],
    
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='joshlin69',
    description='Flask web UI for ROS 2 robot system',
    entry_points={
        'console_scripts': [
            'flask_node = flask_ui.node:main',
        ],
    },
)
