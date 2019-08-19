# apriltag_detection
AprilTag detection for radar odomtery evaluaiton

## Install
The package depends on AprilTag3 library and apriltag_ros ROS wrapper.
Install them in a new workspace by using

```
source /opt/ros/$ROS_DISTRO/setup.bash  # Source your ROS distro 
mkdir -p ~/apriltag_ws/src                # Make a new workspace 
cd ~/apriltag_ws/src                      # Navigate to the source space
git clone https://github.com/AprilRobotics/apriltag.git      # Clone Apriltag library
git clone htps://github.com/AprilRobotics/apriltag_ros.git  # Clone Apriltag ROS wrapper
git clone hhttps://github.com/jurajpersic/apriltag_detection.git  # Clone the parametrized detections package
cd ~/apriltag_ws                          # Navigate to the workspace
rosdep install --from-paths src --ignore-src -r -y  # Install any missing packages
catkin build    # Build all packages in the workspace (catkin_make_isolated will work also)
source ~/apriltag_ws/devel/setup.bash
```

## Usage
In terminal, run
```
roslaunch apriltag_detection apriltag_detect_radar_odometry_v1.launch
```

## Compatibility
Tested on Ubunutu 18.04 and ROS Melodic. Should work with Ubuntu 16.04 and ROS Kinetic.
