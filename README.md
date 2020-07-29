# Leo Rover Vanilla #

1. Clone
```
git clone https://github.com/PUT-UGV-Team/leo-marsyard.git
```
2. Remove /build, /devel and /logs folders if they still exist
```
cd leo_vanilla
cd leo_ws
rm -rf build devel logs
cd ..
```
3. Create a folder in .gazebo and copy model of terrain to gazebo models folder
```
cp -r leo_ws/src/leo/leo_gazebo/models/terrain/* $HOME/.gazebo/models/terrain/
cd $HOME/.gazebo/models
wget --load-cookies /tmp/cookies.txt "https://docs.google.com/uc?export=download&confirm=$(wget --quiet --save-cookies /tmp/cookies.txt --keep-session-cookies --no-check-certificate 'https://docs.google.com/uc?export=download&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J' -O- | sed -rn 's/.*confirm=([0-9A-Za-z_]+).*/\1\n/p')&id=1BkFel0FbazgMBX4ESszcX8o3YBbMmQ6J" -O model4.obj && rm -rf /tmp/cookies.txt

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

