# ros_vrx_path_follow
基于ros的无人艇路径跟踪
```
sudo apt insatll  ros-melodic-robot-localization

sudo apt insatll ros-melodic-velodyne-*

sudo apt insatll ros-melodic-hector-gazebo

mkdir ws

cd ws

git clone https://github.com/bao666liang/ros_vrx_path_follow.git

catkin_make

 source ./devel/setup.bash

roslaunch bringup bringup_simulation.launch

rosrun path_tracking pathFollowing

 //melodic的雷达无效，但noetic可以，gazebo9的问题？
 ```


