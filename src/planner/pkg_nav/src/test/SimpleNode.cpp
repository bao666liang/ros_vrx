#include <iostream>
#include <vector>
#include <queue>
#include <string>
// #include <Eigen/Dense>

struct SimpleNode
{
    int index_x;
    int index_y;
    int index_theta;
    double cost;

    bool operator>(const SimpleNode &right) const
    {
        return cost > right.cost;
    }

    SimpleNode();
    SimpleNode(int x, int y, int theta, double gc, double hc);
};

SimpleNode::SimpleNode()
{
}

SimpleNode::SimpleNode(int x, int y, int theta, double gc, double hc)
    : index_x(x), index_y(y), index_theta(theta), cost(gc + hc)
{
}

int main()
{
    std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> openlist_; //升序队列，小顶堆

    SimpleNode temp1(0, 0, 0, 0, 0);
    temp1.cost = 6.08;
    temp1.index_x = 10;
    openlist_.push(temp1); //它将新元素插入优先队列。

    SimpleNode temp2(0, 0, 0, 0, 0);
    temp2.cost = 7.08;
    temp2.index_x = 11;
    openlist_.push(temp2);

    SimpleNode temp3(0, 0, 0, 0, 0);
    temp3.cost = 6.68;
    temp3.index_x = 12;
    openlist_.push(temp3);
    int len = openlist_.size();
    std::cout << len << std::endl;

    while (!openlist_.empty())
    {
        double temp_cost = openlist_.top().index_x; //top()	此函数用于寻址优先队列的最顶层元素。
        std::cout << temp_cost << std::endl;
        openlist_.pop(); //pop()	它将优先级最高的元素从队列中删除。
    }
    return 0;
}
