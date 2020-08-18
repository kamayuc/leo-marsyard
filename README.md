# Leo Rover Marsyard
## Setup

1. Create new ROS workspace
```
mkdir -p ~/leo_ws/src
```
2. Clone
```
cd ~/leo_ws/src
git clone https://github.com/PUT-UGV-Team/leo-marsyard.git
```
3. Create a folder in .gazebo and copy model of terrain to gazebo models folder
```
cp -r leo_ws/src/leo/leo_gazebo/models/terrain/* $HOME/.gazebo/models/terrain/
cd $HOME/.gazebo/models/terrain/meshes
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J" -O model4.obj && rm -rf /tmp/cookies.txt

```
4. Build
```
cd ..
catkin_make
```
5. Source workspace
```
source devel/setup.bash
```
6. Run Gazebo simulation
```
roslaunch leo_gazebo leo_marsyard.launch
```
7. Open another terminal and run rviz
```
roslaunch leo_viz rviz.launch
```

## Topics

Set velocity (message type: [cmd_vel](http://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html))
* /leo/leo_velocity_controller/cmd_vel

Odometry information:
* /leo/leo_velocity_controller/odom

Leo robot parameters and updates:
* /leo/leo_velocity_controller/parameter_descriptions
* /leo/leo_velocity_controller/parameter_updates


## Dependencies

* [Ubuntu 18.04](https://releases.ubuntu.com/18.04)
* [ROS Melodic Morenia](http://wiki.ros.org/melodic/Installation/Ubuntu)
