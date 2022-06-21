#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from pkg_nav.msg import *
from nav_msgs.msg import Path, OccupancyGrid,Odometry

"""
    需求:
        创建两个ROS 节点，服务器和客户端，
        客户端可以向服务器发送目标数据N(一个整型数据)服务器会计算 1 到 N 之间所有整数的和,
        这是一个循环累加的过程，返回给客户端，这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用 action实现。
    流程:
        1.导包
        2.初始化 ROS 节点
        3.使用类封装，然后创建对象
        4.创建服务器对象
        5.处理请求数据产生响应结果，中间还要连续反馈
        6.spin
"""

class MyActionServer:
    def __init__(self):
        #SimpleActionServer(name, ActionSpec, execute_cb=None, auto_start=True)
        self.server = actionlib.SimpleActionServer("/ow/local_path",local_pathAction,self.cb,False)
        self.server.start() 
        self.local_path = Path()
        rospy.loginfo("The local_path 服务端启动")


    def cb(self,goal):
        rospy.loginfo("The local_path 服务端处理请求:")
        #1.解析目标值
        num = goal.num
        # self.local_path = goal.local_path
        #2.循环累加，连续反馈
        rate = rospy.Rate(1)
        sum = 0
        for i in range(1,num + 1):
            # 累加
            sum = sum + i
            # 计算进度并连续反馈
            feedBack = i*1.0 / num
            rospy.loginfo("当前进度:%.2f/100 ",feedBack*100.0)
            rospy.loginfo("goal.runCommand = %d ",goal.runCommand)
            rospy.loginfo("goal.local_path.size =  ")
            # print(self.local_path.poses[i].pose.position)
            print(goal.local_path.poses[0].pose.position)
            # if i == 8:
              # self.server.set_aborted()

            feedBack_obj = local_pathFeedback()
            feedBack_obj.progress_bar = feedBack
            self.server.publish_feedback(feedBack_obj)
            rate.sleep()
        #3.响应最终结果
        result = local_pathResult()
        result.result = sum        
        self.server.set_succeeded(result)
        # self.server.set_aborted()
        rospy.loginfo("响应结果:%d ",sum)
        
if __name__ == "__main__":
    rospy.init_node("action_server_p")
    server = MyActionServer()
    rospy.spin()