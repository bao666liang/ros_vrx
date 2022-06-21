#!/usr/bin/env python
# -*- coding: utf-8 -*-

from Point import Point
from  Array2D import Array2D

class AStar:
    """
    AStar算法的Python3.x实现
    """
    class Node:  # 描述AStar算法中的节点数据
        def __init__(self, point, endPoint, g=0):
            self.point = point  # 自己的坐标
            self.father = None  # 父节点
            self.g = g  # g值，g值在用到的时候会重新算
            self.h = (abs(endPoint.x - point.x) +
                      abs(endPoint.y - point.y)) * 10  # 计算h值
            self.f = self.g+self.h

    def __init__(self, map2d, startPoint, endPoint, passTag=100):
        """
        构造AStar算法的启动条件
        :param map2d: Array2D类型的寻路数组
        :param startPoint: Point或二元组类型的寻路起点
        :param endPoint: Point或二元组类型的寻路终点
        :param passTag: int类型的可行走标记（若地图数据!=passTag即为障碍）
        """
        # 开启表
        self.openList = []
        # 关闭表
        self.closeList = []
        # 寻路地图
        self.map2d = map2d
        
        # 起点终点
        if isinstance(startPoint, Point) and isinstance(endPoint, Point):
            self.startPoint = startPoint
            self.endPoint = endPoint
        else:
            self.startPoint = Point(*startPoint)
            self.endPoint = Point(*endPoint)

        # 可行走标记
        self.passTag = passTag

    def getMinNode(self):
        """
        获得openlist中F值最小的节点
        :return: Node
        """
        currentNode = self.openList[0]
        for node in self.openList:
            #if node.g + node.h < currentNode.g + currentNode.h:
            if node.f < currentNode.f:
                currentNode = node
        return currentNode

    def pointInCloseList(self, point):
        for node in self.closeList:
            if node.point == point:
                return True
        return False

    def pointInOpenList(self, point):
        for node in self.openList:
            if node.point == point:
                return node
        return None

    def endPointInCloseList(self):
        for node in self.openList:
            if node.point == self.endPoint:
                # print("node.point",node.point)
                return node
        return None

    def searchNear(self, minF,offsetY, offsetX):
        """
        搜索节点周围的点
        :param minF:F值最小的节点
        :param offsetX:坐标偏移量
        :param offsetY:
        :return:
        """
        # 越界检测
        if minF.point.x + offsetX < 0 or minF.point.x + offsetX > self.map2d.w - 1 \
                or minF.point.y + offsetY < 0 or minF.point.y + offsetY > self.map2d.h - 1:
            return
        # 如果是障碍，就忽略
        if self.map2d.data[minF.point.y + offsetY][minF.point.x + offsetX] >= self.passTag:
            return
        
        # 如果在关闭表中，就忽略
        currentPoint = Point(minF.point.x + offsetX, minF.point.y + offsetY)
        if self.pointInCloseList(currentPoint):
            return
        # 设置单位花费
        if offsetX == 0 or offsetY == 0:
            step = 10
        else:
            step = 14

        # 如果不再openList中，就把它加入openlist
        currentNode = self.pointInOpenList(currentPoint)
        if not currentNode:
            currentNode = AStar.Node(
                currentPoint, self.endPoint, g=minF.g + step)
            currentNode.father = minF
            self.openList.append(currentNode)
            #self.tempList.append(currentNode)
            return
        # 如果在openList中，判断minF到当前点的G是否更小
        #self.tempList.append(currentNode)
        if minF.g + step < currentNode.g:  # 如果更小，就重新计算g值，并且改变father
            currentNode.g = minF.g + step
            currentNode.father = minF

    def start(self):
        """
        开始寻路
        :return: None或Point列表（路径）
        """
        # 判断寻路终点是否是障碍
        # print("endPoint")
        # print(self.endPoint.y)
        # print(self.endPoint.x)
        if self.map2d.data[self.endPoint.y][self.endPoint.x] >= self.passTag:
            print("endPoint is not passTag !")  # TODO 代码处理为可通行
            return None

        # 1.将起点放入开启列表
        startNode = AStar.Node(self.startPoint, self.endPoint);
        self.openList.append(startNode);
        self.tempList = [];
        #print(f"self.openList {self.openList}")

        # 2.主循环逻辑
        while True:
            self.tempList.clear();
            # print("self.openList")
            # for node in self.openList:
            #     print(f"{node.point},f:{node.f}")
            # 找到F值最小的点
            minF = self.getMinNode();
            # 把这个点加入closeList中，并且在openList中删除它
            self.closeList.append(minF);
            self.openList.remove(minF);
            # print("self.closeList = ", len(self.closeList))
            # for node in self.closeList:
            #     print(node.point)
            # 判断这个节点的上下左右8 个节点
            self.searchNear(minF, -1, -1);
            self.searchNear(minF, -1, 0);
            self.searchNear(minF, -1, 1);

            self.searchNear(minF, 0, -1);
            self.searchNear(minF, 0, 1);

            self.searchNear(minF, 1, -1);
            self.searchNear(minF, 1, 0);
            self.searchNear(minF, 1, 1);
            # print("self.openList")
            # for node in self.openList:
            #     print(node.point)
            # self.openList.clear()
            # for node in self.tempList:
            #     self.openList.append(node)
            #     #print(node.point)
            # 判断是否终止
            point = self.endPointInCloseList(); #Node
            if point:  # 如果终点在关闭表中，就返回结果
                # print("关闭表中")
                cPoint = point;
                pathList = [];
                while True:
                    if cPoint.father:
                        pathList.append(cPoint.point);
                        cPoint = cPoint.father;
                    else:
                        #print(pathList)
                        # print(list(reversed(pathList)))
                        # print(pathList.reverse())
                        return list(reversed(pathList));
            if len(self.openList) == 0:
                print("self.openList is empty !")
                return None;


if __name__ == '__main__':
    map2d = Array2D(10, 10)
    startPoint = Point(0,0)
    endPoint = Point(9,0)
    astar = AStar(map2d,startPoint, endPoint)
    astar.Node(startPoint,endPoint)
    print(astar)
