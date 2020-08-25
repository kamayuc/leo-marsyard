# Leo Rover Marsyard
## Setup
1. Create new ROS workspace
```
mkdir -p ~/kamayuc/leo_ws/src
```
2. Clone
```
cd ~/kamayuc/leo_ws/src
git clone https://github.com/kamayuc/leo-marsyard.git
```
3. Create a folder in .gazebo and copy model of terrain to gazebo models folder
```
cd
mkdir -p ~/.gazebo/models/terrain
cp -r ~/kamayuc/leo_ws/src/leo-marsyard/leo_gazebo/models/terrain/* $HOME/.gazebo/models/terrain/
cd $HOME/.gazebo/models/terrain/meshes
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J" -O model4.obj && rm -rf /tmp/cookies.txt
```
4. Initialize rosdep tool to install any missing dependencies
```
cd ~/kamayuc/leo_ws
sudo rosdep init
```
5. Install dependencies
```
rosdep update
sudo apt update
rosdep install --from-paths src --ignore-src
```
4. Build
```
cd ~/kamayuc/leo_ws
catkin_make
```
5. Source workspace
```
source devel/setup.bash
```
6. Install Gazebo Pluggins
```
sudo apt-get install ros-melodic-gazebo-plugins
sudo apt-get install ros-melodic-hector-gazebo-plugins
```
7. Run Gazebo simulation
```
roslaunch leo_gazebo leo_gazebo.launch
```
8. Open another terminal and run rviz
```
source devel/setup.bash
roslaunch leo_viz rviz.launch
```

## Topics
Set velocity (message type: [cmd_vel](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))
* /cmd_vel

Odometry information: (message type: [odom](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/TwistStamped.html))
* /wheel_odom

Camera information: (message type: [camera_info](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/CameraInfo.html))
* /camera/camera_info
* /zed/camera/left/camera_info
* /zed/camera/right/camera_info

Camera data: (message type: [image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Image.html))
* /camera/image_raw
* /zed/camera/left/image_raw
* /zed/camera/right/image_raw

IMU data: (message type: [image](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/Imu.html))
* /imu/data
* /imu/data/bias

Magnetometer data: (message type: [image](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Vector3Stamped.html))
* /magnetic/data


Leo robot parameters and updates:
* /controllers/diff_drive/parameter_descriptions
* /controllers/diff_drive/parameter_updates

## Configurations
Magnetometer & IMU [reference](http://wiki.ros.org/hector_gazebo_plugins).

## Dependencies

* [Ubuntu 18.04](https://releases.ubuntu.com/18.04)
* [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
* [Gazebo9](http://gazebosim.org/tutorials/?tut=ros_wrapper_versions)
