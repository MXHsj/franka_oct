# Autonomous OCT Scanning using Franka Emika Panda

## Description
ROS package for Franka OCT scanning 

Franka controller: ```franka_controllers```

## Usage:
### preparation
- bring robot to home configuration
  ```
  rolaunch franka_oct move_to_start.launch
  ```
- bring up robot
  ```
  roslaunch franka_oct franka_OCT_scan.launch
  ```

### scan along hard-coded path
- do scan w/ pure translation
  ```
  rosrun franka_oct DoTranslationalScan.py (deprecated)
  ```
- do scan w/ rotation + translation

  set ```pureTranslation``` to ```True```
  ```
  rosrun franka_oct DoScan
  ```

### scan manually defined ROI in realsense camera view
- generate scan path from realsense view
  ```
  rosrun franka_oct generate_scan_path.py
  ```
- do scan
  ```
  rosrun franka_oct scan_planner.py
  ```

### scan automatically defined ROI in camera view
- TODO

## Published Topics
- ```OCT_scan_path_starts``` 
  - Type: Float64MultiArray

- ```OCT_scan_path_length```
  - Type: Float64

- ```OCT_scan_flag```
  - Type: Int8
