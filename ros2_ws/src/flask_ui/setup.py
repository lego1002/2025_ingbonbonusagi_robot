from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'flask_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ("share/ament_index/resource_index/packages", ["resource/flask_ui"]),
    ("share/flask_ui", ["package.xml"]),
    
    (os.path.join("share", "flask_ui", "templates"),
     glob("flask_ui/templates/*")),
    
    (os.path.join("share", "flask_ui", "static"),
     glob("flask_ui/static/**/*", recursive=True)),
    ],
    
    install_requires=['setuptools', 'flask'],
    zip_safe=True,
    maintainer='joshlin69',
    maintainer_email='lyou09134@gmail.com',
    description='Flask web UI integraged with ROS2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "flask_node = flask_ui.node:main",
        ],
    },
)
