# ROS Nodes

These nodes are intended to run on the rover's SBC. 

## Build Instructions
1. Ensure ROS2 Humble Hawksbill is installed on your machine.
2. Open this folder in a command prompt.
3. Run `colcon build`. The output should look something like as follows:
```
Starting >>> pico_relay
--- stderr: pico_relay                   
/usr/lib/python3/dist-packages/setuptools/command/install.py:34: SetuptoolsDeprecationWarning: setup.py install is deprecated. Use build and pip and other standards-based tools.
  warnings.warn(
---
Finished <<< pico_relay [1.21s]

Summary: 1 package finished [1.47s]
  1 package had stderr output: pico_relay
```

## Run Instructions (Linux)
1. Make sure you have followed the build instructions without errors.
2. Open this folder in a command prompt.
3. Run `. install/setup.sh`.
4. Navigate to the `launch` directory.
5. Run `ros2 launch web_relay_launch.py`.

## Run Instructions (Windows)
1. Make sure you have followed the build instructions without errors.
2. Open this folder in a command prompt.
3. Run `./install/setup.ps1`.
4. Navigate to the `launch` directory.
5. Run `ros2 launch web_relay_launch.py`.
