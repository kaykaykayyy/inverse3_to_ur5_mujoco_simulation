# README for haply_to_mujoco_ur5

## Project Overview

The `haply_to_mujoco_ur5` project implements a teleoperation system that allows real-time control of a UR5 robot arm using the Haply Inverse3 as haptic input inside the MuJoCo simulation.

## Video Demo

Here is a demo of my project: 

https://raw.githubusercontent.com/kaykaykayyy/inverse3_to_ur5_mujoco_simulation/main/teleop_go_down.mp4

## Features

- Integration with Haply Inverse3 for haptic feedback.
- Utilizes MuJoCo for realistic physics simulation.
- Inverse kinematics calculations for accurate joint positioning.

## Project Structure

```
haply_to_mujoco_ur5
├── haply
│   ├── haply_interface.py
├── HaplyInverse3_testing
│   ├── 1_position_printing.py
│   ├── 2_calibration_testing.py
│   ├── 3_joint_output.py
├── robots
│   ├── mujoco_connection.py
│   ├── ur5_ik.py
├── teleop
│   ├── haply_calibrator.py
├── requirements.txt
├── main.py
├── 20260615_main.py
└── README.md
```

## Equipment Used 
1. Haply Robotics Inc Haply Inverse3 1487 Amerigo L. Majoris 
2. Haply Robotics Inc VerseGrip 1356VGS

## Installation

1. Clone the repository:
   ```
   git clone <repository-url>
   cd haply_to_mujoco_ur5
   ```

2. Install the required dependencies:
   ```
   conda create --name <env> --file <this file>
   platform: win-64
   ```

## Usage

1. Connect the Haply Inverse3 device.
2. Run the main teleoperation script:
   ```
   python -m haply.main
   ```
or you can first try the testing scripts to make sure the haply inverse3 is working as expected 


## Contributing

Contributions are welcome! Please open an issue or submit a pull request for any improvements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.
