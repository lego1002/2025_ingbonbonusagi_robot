from setuptools import find_packages, setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera_launch.py']),
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
            'camera_node = camera.camera_node:main',
            'undistort = camera.undistort:main',
            'sobel = camera.sobel:main',
            'findLine = camera.findLine:main',
            'liquid_level = camera.liquid_level:main',
            'liquid_level_adj = camera.liquid_level_adj:main',
            'canny = camera.canny:main',
            'noise = camera.noise:main',
        ],
    },
)