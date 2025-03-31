

# TimelapseSensor

This project utilizes an IR proximity sensor to trigger a camera shutter, facilitating time-lapse photography. It employs a Xiao BLE module and a MAX30102 IC to detect motion and communicate with a Sony camera over Bluetooth Low Energy (BLE) using the GATT profile.

![Alt Text](https://github.com/mohit3112/TimelapseSensor/blob/main/timelapse3dprinting.gif)

## Features
- **Proximity Detection**: Utilizes a MAX30102 IR sensor.
- **BLE Communication**: Uses Xiao BLE module to interface with Sony cameras.
- **Calibration**: Automatic 20-second calibration period on startup.
- **Mounting**: Designed to attach to the X-axis as shown in the image.

## Getting Started

### Prerequisites
- Hardware: Xiao BLE Module, MAX30102 IR Sensor, Sony Camera.
- Software: nrf connect SDK

### Installation
1. Clone the repository:
    ```bash
    git clone https://github.com/mohit3112/TimelapseSensor.git
    ```
    
### Usage
- Attach the sensor module to the camera's X-axis.
- Ensure the BLE module is properly connected.
- Power on the device and wait for the 20-second calibration to complete.
- Start capturing time-lapse photos once calibration is done.

## Contributing
1. Fork the repository.
2. Create a new branch (`git checkout -b feature-branch`).
3. Commit your changes (`git commit -m 'Add new feature'`).
4. Push to the branch (`git push origin feature-branch`).
5. Open a pull request.

## License
This project is licensed under the MIT License.

## Acknowledgements
- Inspiration from various DIY camera trigger projects.

---

For more details, visit the [TimelapseSensor GitHub repository](https://github.com/mohit3112/TimelapseSensor).

---
