#include "ros/ros.h"
#include "std_msgs/Float64.h"

class MyPlus
{
public:
    MyPlus()
    {
        value = 0.0;
    }
    void onInit()
    {
        //获取 NodeHandle
        ros::NodeHandle nh;
        //创建发布与订阅对象
        pub = nh.advertise<std_msgs::Float64>("out", 100);
        sub = nh.subscribe<std_msgs::Float64>("in", 100, &MyPlus::doCb, this);
    }
    //回调函数
    void doCb(const std_msgs::Float64::ConstPtr &p)
    {
        double num = p->data;
        //数据处理
        double result = num + value;
        std_msgs::Float64 r;
        r.data = result;
        //发布
        pub.publish(r);
    }

private:
    ros::Publisher pub;
    ros::Subscriber sub;
    double value;
};

int main(int argc, char *argv[])
{
    //执行 ros 节点初始化
    ros::init(argc, argv, "ow_decision_node");
    ROS_INFO("The program of decision_node is running ...");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    MyPlus myPlus;
    ROS_INFO("hello world!");
    ros::spin();

    return 0;
}
