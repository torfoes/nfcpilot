from setuptools import setup
import sys
import platform

package_name = 'stepper_motor_controller'

if platform.system() == 'Linux' and 'arm' in platform.machine():
    install_requires = [
        'setuptools',
        'RPi.GPIO',
    ]
else:
    install_requires = [
        'setuptools',
    ]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=install_requires,
    zip_safe=True,
    maintainer='karlos',
    maintainer_email='karloszuru@gmail.com',
    description='ROS 2 package for controlling stepper motors via DRV8825 drivers using RPi.GPIO',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = stepper_motor_controller.motor_controller_node:main',
        ],
    },
)
