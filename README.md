# nfcpilot

**nfcpilot** is an open-source reel-to-reel NFC tag programmer designed to automate the programming of NFC tags using stepper motors and an NFC reader.

## Nodes

- **nfc_reader_node**: Reads NFC tags and publishes the tag data to a ROS 2 topic.

- **motor_controller_node**: Controls the stepper motors that drive the reel-to-reel mechanism. It can start, stop, and adjust the speed of the motors based on received commands.

- **orchestrator_node**: Coordinates the NFC reader and motor controller. It listens for NFC tag detections and instructs the motor controller to start or stop the motors accordingly.

---

This setup allows for efficient and automated programming of NFC tags in a continuous reel-to-reel process.

# Getting Started

### You may need this system dependency for pyscard
```commandline
sudo apt install libpcsclite-dev
```

```commandline
python3 -m venv venv
touch venv/COLCON_IGNORE
source venv/bin/activate

export PYTHONPATH="$(pwd)/venv/lib/python3.12/site-packages:$PYTHONPATH"
pip install -e .

colcon build --symlink-install
source install/setup.bash


```

To run a launch a motor node
```commandline
ros2 run stepper_motor_controller motor_controller_node
```
