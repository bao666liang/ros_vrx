#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Path, OccupancyGrid,Odometry
from geometry_msgs.msg import PoseStamped, Quaternion,Twist,Pose
import tf
import math
from Array2D import Array2D
from Point import Point
from AStar import AStar
import numpy as np
from enum import Enum
import actionlib
from pkg_nav.msg import *

# 当前运行状态
class RunState(Enum):
    IN_IDLE = 0
    IN_initial = 1
    IN_wait_command = 2
    IN_global_map = 3
    IN_global_plan = 4
    IN_local_map = 5
    IN_local_plan = 6
    IN_emergency_stop = 7

# 触发恢复模式
class RecoveryTrigger(Enum):
    IN_normal = 0  #ok
    IN_global_map_RT = 1  # 获取全局地图失败
    IN_global_plan_RT = 2  # 全局路径规划失败
    IN_local_map_RT = 3  # 获取局部地图失败
    IN_local_plan_RT = 4  # 局部路径规划失败
    IN_oscillation_RT = 5  # 长时间困在一片小区域

# 起始运动状态
x, y, th = 0, 0, 0


class Decision:
    """decision module Class
    """

    def __init__(self):
        self.global_map = OccupancyGrid()  # global map for global planner
        self.bHave_global_map = False
        self.local_map = OccupancyGrid()  # local map for local planner
        self.bHave_local_map = False

        self.local_path = Path()  # local path record
        self.bHave_local_path = False
        self.global_path = Path()  # global path record
        self.bHave_global_path = False

        self.odometry = Odometry()
        self.bhave_odometry = False
        self.act_pose = Pose()  # actual pose :  Pose
        self.act_vel = Twist()  # actual velocity :   Twist
        self.goal_pose =  Pose() # goal pose : Pose
        self.bHave_goal_pose = False

        self.e_runState = RunState.IN_IDLE 
        self.recoverBehavior_state = RecoveryTrigger.IN_normal

        self.run_state = 0

    def global_map_cb(self, global_map):
        """
        get global map callback
        """
        rospy.loginfo("The global_map_cb is loading global_map ...")
        # self.bHave_global_map = False

        current_time = rospy.Time.now()  # 时间戳
        self.global_map_height = global_map.info.height  # 长和宽 mm
        self.global_map_width = global_map.info.width
        self.resolution = global_map.info.resolution
        rospy.loginfo("The global_map_height  =  %d,The global_map_width  =  %d ",
                      self.global_map_height, self.global_map_height)

        if self.global_map_height*self.global_map_width > 0:
            self.global_map = global_map
            self.bHave_global_map = True
            rospy.loginfo("The global_map_cb has loaded global_map !")
        else:
            rospy.logfatal(
                "The global_map is invalid in global_map_cb. The global_map of height or width is equal 0 !")

    def local_map_cb(self, local_map):
        """
        get local map callback
        """
        rospy.loginfo("The local_map_cb is loading local_map ...")
        self.bHave_local_map = False
        current_time = rospy.Time.now()  # 时间戳
        self.local_map_height = local_map.info.height  # 长和宽 mm
        self.local_map_width = local_map.info.width
        rospy.loginfo("The local_map_height  =  %d,The local_map_width  =  %d ",
                      self.local_map_height, self.local_map_width)
        if self.local_map_height * self.local_map_width > 0:
            self.local_map = local_map
            self.bHave_local_map = True
            rospy.loginfo("The local_map_cb has loaded local_map !")
        else:
            rospy.logfatal(
                "The local_map is invalid in local_map_cb. The local_map of height or width is equal 0 !")

    def global_path_update(self, path_pub):
        """
        global path update function
        """
        rospy.loginfo("The global_path_update is planing ...")
        rospy.loginfo(" self.bHave_global_map = %d", self.bHave_global_map)
        rospy.loginfo("The height*width of  self.global_map= %d: ",
                      self.global_map_height*self.global_map_width)

        if not self.bHave_global_map or self.global_map_height*self.global_map_width < 100*100:
            rospy.logerr(
                "The global_path_update has a Exception : has not accepted global_map !")

        # 实例化地图数组
        # return #TODO
      
        self.run_state = RunState.IN_global_plan
        map2d = Array2D(self.global_map_height, self.global_map_width)
        map2d.data = np.array(self.global_map.data).reshape(
            self.global_map_height, self.global_map_width)
        aStar = AStar(map2d, Point(0, 0), Point(60, 60))  # TODO 起点和终点

        # start finding global path
        rospy.loginfo("-------Finding path---------")
        start_time = rospy.Time.now()  # start time
        pathList = aStar.start()

        if pathList == None:
            rospy.logerr(
                "The global_path_update has a Exception : finding global path failed !")
            self.recoverBehavior_state = RecoveryTrigger.IN_global_plan_RT
            return False

        # 配置路径
        self.global_path.header.stamp = rospy.Time.now()
        self.global_path.header.frame_id = 'map'
        for point in pathList:  # 遍历路径点
            map2d[point.x][point.y] = 7  # 在map2d上以'7'显示路径
            # print(point)
            x = point.x*self.resolution
            y = point.y*self.resolution

            theta = 0  # TODO
            quat = tf.transformations.quaternion_from_euler(0, 0, theta)  # 四元素
            pose = PoseStamped()  # 配置姿态
            pose.header.stamp = rospy.Time.now()  # 时间戳
            pose.header.frame_id = 'map'
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.x = quat[0]
            pose.pose.orientation.y = quat[1]
            pose.pose.orientation.z = quat[2]
            pose.pose.orientation.w = quat[3]

            self.global_path.poses.append(pose)
            if len(self.global_path.poses) > 5000:  # 路径数量限制
                self.global_path.poses.pop(0)

        # 发布路径
        rospy.loginfo("----------The path has found !------------")
        map2d.showArray2D()  # 再次显示地图
        end_time = rospy.Time.now()
        t_findpath = (end_time-start_time)
        rospy.loginfo("The total of find golbal path :  %.2fms ", t_findpath.to_sec())
        path_pub.publish(self.global_path)

    def local_path_update(self, path_pub):
        """
        local path update function
        """
        rospy.loginfo("The local_path_update is running ...")
        global x, y, th

        # 发布tf
        tf_br = tf.TransformBroadcaster()
        tf_br.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0),
                            rospy.Time.now(), "odom", "map")
        # 配置运动
        dt = 0.1  # 1 / 50
        vx = 2  # 0.25
        vy = 1
        vth = 0
        delta_x = (vx * math.cos(th) - vy * math.sin(th)) * dt
        delta_y = (vx * math.sin(th) + vy * math.cos(th)) * dt
        delta_th = vth * dt
        x += delta_x
        y += delta_y
        th += delta_th
        # 四元素转换
        quat = tf.transformations.quaternion_from_euler(0, 0, th)

        # 时间戳
        current_time = rospy.Time.now()
        # 配置姿态
        pose = PoseStamped()
        pose.header.stamp = current_time
        pose.header.frame_id = 'odom'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        # 配置路径
        self.local_path.header.stamp = current_time
        self.local_path.header.frame_id = 'odom'
        self.local_path.poses.append(pose)

        # 路径数量限制
        if len(self.local_path.poses) > 1000:
            self.local_path.poses.pop(0)

        # 发布路径
        path_pub.publish(self.local_path)

    def odometry_cb(self, odometry):
        rospy.loginfo("The odometry_cb has got odometry ...")
        # rospy.loginfo(
        #     "The act_pose x : %.2fm, y = %.2fm, theta = %.2frad")
        # self.act_pose =  
        # self.bhave_odometry = 
        pass

    def goal_pose_cb(self):
        rospy.loginfo("The goal_pose_cb is running ...")
        self.goal_pose = [0, 0, 0]  # x,   y,   z
        self.bHave_goal_pose = True
        pass

    def done_cb(self,state,result):
        if state == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("响应结果:%d",result.result)
        else:
          rospy.logerr("响应结果处理失败 ！%d",state) 
      
    def active_cb(self):
        rospy.loginfo("服务被激活....")

    def feedback_cb(self,feedback):
        rospy.loginfo("当前进度:%.2f/100",feedback.progress_bar*100)

    def node_start(self):
        """
        节点启动函数
        """
        try:
            # 初始化节点
            rospy.init_node('ow_decision_node')
            rospy.loginfo("The program of decision_node is running ...")

            # Subscriber global map 
            self.map_sub = rospy.Subscriber("/ow/global_map", OccupancyGrid,
                                              self.global_map_cb, queue_size=1)
            # Subscriber local map 
            self.map_sub = rospy.Subscriber("/ow/local_map", OccupancyGrid,
                                            self.local_map_cb, queue_size=1)

            # Subscriber include the  current  Pose and Twist  message of Odometry
            self.odometry_sub = rospy.Subscriber(
                "/ow/odometry", Odometry, self.odometry_cb)
            # ow goal pose Subscriber, Topic ：/ow/goal_pose
            self.goal_pose_sub = actionlib.SimpleActionClient(
                "/ow/goal_pose", PoseStamped, self.goal_pose_cb)

            # Publisher global path 0.1hz
            self.global_path_pub = rospy.Publisher(
                '/ow/global_path', Path, queue_size=10)
            # Publisher local path
            self.local_path_pub = rospy.Publisher(
                '/ow/local_path', Path, queue_size=50)
            
            # 3.创建 action Client 对象
            self.local_path_client = actionlib.SimpleActionClient("/ow/local_path",local_pathAction)
                # 4.等待服务
            self.local_path_client.wait_for_server() 
                # 5.组织目标对象并发送
            goal_obj = local_pathGoal()  #
            goal_obj.num = 10 #Header
            self.local_path_client.send_goal(goal_obj,self.done_cb,self.active_cb,self.feedback_cb)
            
            # initial cycle rate 0.1hz
            rate = rospy.Rate(10)
            # 在程序没退出的情况下
            while not rospy.is_shutdown():
                # 数据更新函数
                # print(self.bHave_global_map)
                if self.bHave_global_map:
                  self.global_path_update(self.global_path_pub)
                if self.bHave_local_map:
                    self.local_path_update(self.local_path_pub)
                # 休眠
                rate.sleep()
            rospy.spin()

        except Exception as e:
            rospy.logfatal("decision_node has a Exception : %s", e)  # [FATAL]
            # rospy.logerr("程序异常终止！%s", e)  # [ERROR] [error code] RED
            # # rospy.loginfo("程序异常终止！%s", e)  # [INFO]

if __name__ == '__main__':
    decision = Decision()
    decision.node_start()
