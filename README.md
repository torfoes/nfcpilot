# nfcpilot

**nfcpilot** is an open-source reel-to-reel NFC tag programmer designed to automate the programming of NFC tags using stepper motors and an NFC reader.

## Nodes

- **nfc_reader_node**: Reads NFC tags and publishes the tag data to a ROS 2 topic.

- **motor_controller_node**: Controls the stepper motors that drive the reel-to-reel mechanism. It can start, stop, and adjust the speed of the motors based on received commands.

- **orchestrator_node**: Coordinates the NFC reader and motor controller. It listens for NFC tag detections and instructs the motor controller to start or stop the motors accordingly.

---

This setup allows for efficient and automated programming of NFC tags in a continuous reel-to-reel process.
