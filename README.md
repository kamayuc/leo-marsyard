# Leo Rover Vanilla #

1. Clone
```
git clone https://fsmuggler@bitbucket.org/marsrover-framework/leo-marsyard.git
```
2. Remove /build, /devel and /logs folders if they still exist
```
cd leo_vanilla
cd leo_ws
rm -rf build devel logs
```
3. Copy model of terrain to gazebo models folder
```

```
4. Build workspace
```
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

