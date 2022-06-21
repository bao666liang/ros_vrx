#ifndef ASTAR_UTILITY_H
#define ASTAR_UTILITY_H

#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>
using namespace std;


class Node //描述AStar算法中的节点数据
{
public:
    geometry_msgs::Point point;
    int gCost;
    int hCost;
    int fCost;               //fCost = gCost + hCost
    Node *parentNode;        //父节点 NULL

    Node(geometry_msgs::Point currentPoint, geometry_msgs::Point goalPoint, int gCost = 0)
    {
        this->point = currentPoint; //自己的坐标
        this->parentNode = nullptr;    //父节点 //NULL
        this->gCost = gCost;        //g值，g值在用到的时候会重新算
        this->hCost = (abs(goalPoint.x - currentPoint.x) + abs(goalPoint.y - currentPoint.y))*10;
        this->fCost = this->gCost + this->hCost;
        //计算h值
    }
    Node()
    {
        ;
    }

    // bool operator>(const Node &node) const
    // {
    //     return this->fCost > node.fCost;
    // }

    struct compare
    {
        bool operator()(Node *&topNode, Node *&newNode) const
        {
            return topNode->fCost > newNode->fCost;
        }
    };
};

/*
        说明：
            1.构造方法需要两个参数，即二维数组的宽和高
            2.成员变量w和h是二维数组的宽和高
            3.使用：‘对象[x][y]’可以直接取到相应的值
            4.数组的默认值都是0
 */
class Array2D
{
public:
    int length;
    int width;
    vector<vector<int>> data;

public:
    // vector<vector<int>> data;
    Array2D(int length, int width)
    {
        this->length = length;
        this->width = width;
        data.resize(length);
        for (int row = 0; row < length; row++)
        {
            data[row].resize(width);
        }
        for (int row = 0; row < length; row++)
        {
            for (int col = 0; col < width; col++)
                data[row][col] = 0;
        }
    }

    void showArray2D()
    {
        for (int row = 0; row < this->length; row++)
        {
            for (int col = 0; col < this->width; col++)
                std::cout << data[row][col] << " ";
            std::cout << std::endl;
        }
    }
};

// int main(int argc, char *argv[])
// {
//     setlocale(LC_ALL, "");
//     //执行 ros 节点初始化
//     ros::init(argc, argv, "Array2D_node");
//     ROS_INFO("The program of Array2D_node is running ...");
//     //创建 ros 节点句柄(非必须)
//     ros::NodeHandle nh;
//     Array2D map2d(10, 10);
//     cout << map2d.length << "  " << map2d.width << endl;
//     map2d.showArray2D();
//     ros::spin();
//     return 0;
// }
#endif