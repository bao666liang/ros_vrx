# 决策模块处理节点

节点名称： ow_decision_node

## 1. 功能描述
  ###   1.1 订阅全局地图和局部地图
  ###   1.2 发布全局路径和局部路径

## 2. Subscribed Topics

### 2.1 /ow/odometry Odometry

      Describe : Subscribed the  Pose and Twist message of Odometry type
      Topic : /ow/odometry
      Message type: nav_msgs/Odometry

### 2.2 /ow/global_map OccupancyGrid

      Describe : Subscribed  global map message of OccupancyGrid type
      Topic : /ow/global_map
      Message type: nav_msgs/OccupancyGrid
      Rate : 0.1hz

### 2.3 /ow/local_map OccupancyGrid

      Describe : Subscribed  local map message of OccupancyGrid type
      Topic : /ow/local_map
      Message type: nav_msgs/OccupancyGrid
      Rate : 1hz

## 3. Published Topics

### 3.1 /ow/global_path Path

    Describe : Subscribed  global path message of Path type
    Topic : /ow/global_path
    Message type: nav_msgs/Path
    Rate : 0.1hz

### 3.2 /ow/local_path Path

    Describe : Subscribed  global path message of Path type
    Topic : /ow/global_path
    Message type: nav_msgs/Path
    Rate : 1hz

## 4. Services

### 4.1 /ow/goal_pose PoseStamped

    Describe : As a client  that get goal pose message of PoseStamped type of  the server published topics.
    A goal for ow_decision_node to pursue in the world.
    Topic : /ow/goal_pose
    Message type: geometry_msgs/PoseStamped

     A goal for ow_decision_node to pursue in the world.
    参考：move_base/goal (move_base_msgs/MoveBaseActionGoal)

## 5. Action API

### 5.1 Action Published Topics

#### 5.1.1 /ow/local_path local_pathAction

    # Define the goal
    nav_msgs/Path local_path
    int32 num #test -cyb
    ---
    # Define the result
    int8  runResult  #  -1：处理失败  0 : 未处理     1：处理成功
    int32 result #test -cyb
    ---
    #Define a feedback message
    float64 percent_complete  # [0-1]   处理进度
    int8    runState  #  -1：处理失败  0 : 未处理     1：正在处理   2：处理完成
    float64 progress_bar#test -cyb

    参考：
    move_base/feedback (move_base_msgs/MoveBaseActionFeedback)
    Feedback contains the current position of the base in the world.
    move_base/status (actionlib_msgs/GoalStatusArray)
    Provides status information on the goals that are sent to the move_base action.
    move_base/result (move_base_msgs/MoveBaseActionResult)
    Result is empty for the move_base action.

## 6 消息类型

### 6.1 from nav_msgs.msg import Path, OccupancyGrid,Odometry

#### 6.1.1 nav_msgs/Path

    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    geometry_msgs/PoseStamped[] poses由一系列点组成的数组
      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
      geometry_msgs/Pose pose
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z

        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w

#### 6.1.2 nav_msgs/OccupancyGrid

      std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    #--- 地图元数据
      nav_msgs/MapMetaData info
        time map_load_time
        float32 resolution
        uint32 width
        uint32 height
        geometry_msgs/Pose origin
          geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
      int8[] data #地图内容数据，数组长度 = width * height

#### 6.1.3 nav_msgs/Odometry

    std_msgs/Header header
      uint32 seq
      time stamp
      string frame_id
    string child_frame_id
    geometry_msgs/PoseWithCovariance pose
      geometry_msgs/Pose pose  #里程计位姿
        geometry_msgs/Point position
          float64 x
          float64 y
          float64 z
        geometry_msgs/Quaternion orientation
          float64 x
          float64 y
          float64 z
          float64 w
      float64[36] covariance
    geometry_msgs/TwistWithCovariance twist
      geometry_msgs/Twist twist   #速度
        geometry_msgs/Vector3 linear
          float64 x
          float64 y
          float64 z
        geometry_msgs/Vector3 angular
          float64 x
          float64 y
          float64 z
    #协方差矩阵
      float64[36] covariance

### 6.2 from geometry_msgs.msg import PoseStamped, Quaternion, Pose, Twist

#### 6.2.1 geometry_msgs/PoseStamped

    Header header
      uint32 seq
      time stamp（int32 ）
      string frame_id
    Pose pose
      Point position
        float64 x
        float64 y
        float64 z
      Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w

#### 6.2.2 geometry_msgs/PoseWithCovarianceStamped

        std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
        geometry_msgs/PoseWithCovariance pose
        geometry_msgs/Pose pose
            geometry_msgs/Point position
            float64 x
            float64 y
            float64 z
            geometry_msgs/Quaternion orientation
            float64 x
            float64 y
            float64 z
            float64 w
        float64[36] covariance

#### 6.2.3 geometry_msgs/Quaternion

    Quaternion quaternion
      float64 x
      float64 y
      float64 z
      float64 w

#### 6.2.4 geometry_msgs/Pose

    Point position
      float64 x
      float64 y
      float64 z
    Quaternion orientation
      float64 x
      float64 y
      float64 z
      float64 w

#### 6.2.5 geometry_msgs/Twist

    geometry_msgs/Vector3 linear
        float64 x
        float64 y
        float64 z
      geometry_msgs/Vector3 angular
        float64 x
        float64 y
        float64 z

### 6.3 import tf

    tf/tfMessage
      geometry_msgs/TransformStamped[] transforms   #包含了多个坐标系相对关系数据的数组
        std_msgs/Header header
          uint32 seq
          time stamp
          string frame_id
        string child_frame_id
        geometry_msgs/Transform transform
          geometry_msgs/Vector3 translation
            float64 x
            float64 y
            float64 z
          geometry_msgs/Quaternion rotation
            float64 x
            float64 y
            float64 z
            float64 w

###  安装包依赖
    sudo apt-get install ros-melodic-autoware-config-msgs 
  18.04安装osqp-eigen
  1.首先安装依赖osqp和Eigen3
    git clone --recursive https://github.com/oxfordcontrol/osqp
    cd osqp
    mkdir build
    cd build
    cmake .. -DBUILD_SHARED_LIBS=ON
    make -j6
    sudo make install
  2.安装osqp-eigen
    git clone https://github.com/robotology/osqp-eigen.git
    cd osqp-eigen
    mkdir build && cd build
    cmake ../	#默认安装在/usr/local/include中,非apt安装的包，不在/usr/include/下，而是/usr/local/include下
    make
    sudo make install

    sudo apt-get install ros-melodic-geographic-msgs 

