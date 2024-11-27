 
import os
from glob import glob
from setuptools import setup

package_name = 'hexapod_robot'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # Install marker file for the package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Package xml file
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch',
         glob('launch/*launch.[pxy][yma]*')),
        # Config files
        ('share/' + package_name + '/config',
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Slava Karpizin',
    maintainer_email='karpizin@gmail.com',
    description='ROS2 package rc control any type of hardware',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_driver_node = hexapod_robot.servo_driver_node:main',
            'ppm_controller_node = hexapod_robot.ppm_controller_node:main',
            'sbus_controller_node = hexapod_robot.sbus_controller_node:main',
            'keyboard_controller = hexapod_robot.keyboard_controller:main',
            'joystick_controller = hexapod_robot.joystick_controller:main',
        ],
    },
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development :: Libraries',
        'Topic :: Scientific/Engineering',
    ]
)
