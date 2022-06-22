# ros_vrx

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
 
 //osqp求解器安装后将.so文件拷贝到链接库
 ```


