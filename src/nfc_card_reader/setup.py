from setuptools import setup

package_name = 'nfc_card_reader'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'pyscard',
    ],
    zip_safe=True,
    maintainer='karlos',
    maintainer_email='karloszuru@gmail.com',
    description='ROS 2 package for reading NFC cards',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nfc_reader_node = nfc_card_reader.nfc_reader_node:main',
        ],
    },
)
