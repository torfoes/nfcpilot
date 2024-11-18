from setuptools import setup
import os
from glob import glob

package_name = 'orchestrator_node'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karlos',
    maintainer_email='karloszuru@gmail.com',
    description='An orchestrator node that coordinates NFC reader and motor controller nodes.',
    license='MIT License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'orchestrator_node = orchestrator_node.orchestrator_node:main',
        ],
    },
)
