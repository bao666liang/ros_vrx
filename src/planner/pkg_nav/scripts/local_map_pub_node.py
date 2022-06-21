#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, OccupancyGrid
import numpy as np


def DataUpdating(map_pub, local_map, map_height, map_width):
    """
    数据更新函数
    """
    # 时间戳
    current_time = rospy.Time.now()

    local_map.info.map_load_time = current_time
    local_map.header.frame_id = "map"
    local_map.info.resolution = 1  # 分辨率 1m  图片分片率(单位: m/像素)。
    local_map.info.height = map_height  # 长和宽 mm
    local_map.info.width = map_width
    local_map.info.origin.position.x = 0  # 原点位置
    local_map.info.origin.position.y = 0
    local_map.info.origin.position.z = 0
    local_map.info.origin.orientation.x = 0  # 原点姿态
    local_map.info.origin.orientation.y = 0
    local_map.info.origin.orientation.z = 0
    local_map.info.origin.orientation.w = 1

    # 默认全部未知区域  # 全部为-1 -> 0
    # local_map.data = [-1] * local_map.info.width * local_map.info.height
    shape = local_map.info.width * local_map.info.height
    # print("local_map.data.shape = ", shape)
    local_map.data = np.full(shape, 0, dtype=int, order='C')
    # print(type(local_map.data))

    # # 设置障碍区域 x
    boundary = 10  #10
    # for y in [boundary, local_map.info.height - boundary]:  # 行索引
    #     for x in range(boundary, local_map.info.width - boundary):  # 列索引
    #         local_map.data[x + y*local_map.info.width] = 100

    # for y in range(boundary, local_map.info.height - boundary):  # 行索引
    #     for x in [boundary, local_map.info.width - boundary-1]:  # 列索引
    #         local_map.data[x + y*local_map.info.width] = 100

    # # 设置可行区域
    # for y in range(boundary+1, local_map.info.height - boundary-1):  # 行索引
    #     for x in range(boundary+1, local_map.info.width - boundary-1):  # 列索引
    #         local_map.data[x + y*local_map.info.width] = 0

    # 设置障碍区域
    for y in [boundary*2]:  # 行索引
        for x in range(0, local_map.info.width-boundary*2):  # 列索引
            local_map.data[x + y*local_map.info.width] = 100

    for y in [local_map.info.height - boundary*3]:  # 行索引
        for x in range(boundary*2, local_map.info.width):  # 列索引
            local_map.data[x + y*local_map.info.width] = 100

    # 发布地图
    map_pub.publish(local_map)


def node():
    """
    节点启动函数
    """
    try:
        # 初始化节点MapPublish
        rospy.init_node('ow_local_map_publish_node')
        # 定义发布器 map_pub 发布 local_map
        map_pub = rospy.Publisher('/ow/local_map', OccupancyGrid, queue_size=1)

        # 定义地图
        local_map = OccupancyGrid()
        map_height = 100
        map_width = 100

        rospy.loginfo("The program of ow_local_map_publish_node is running ...")
        # 初始化循环频率
        rate = rospy.Rate(1)
        # 在程序没退出的情况下
        while not rospy.is_shutdown():
            # 数据更新函数
            DataUpdating(
                map_pub, local_map, map_height, map_width)
            rate.sleep()
    except Exception as e:
        rospy.logfatal("map_publish_node has a Exception : %s", e)  # [FATAL]


if __name__ == '__main__':
    node()
