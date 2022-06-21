#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "pkg_nav/AddIntsAction.h"
/*  
    需求:
        创建两个ROS节点，服务器和客户端，
        客户端可以向服务器发送目标数据N（一个整型数据）
        服务器会计算1到N之间所有整数的和，这是一个循环累加的过程，返回给客户端，
        这是基于请求响应模式的，
        又已知服务器从接收到请求到产生响应是一个耗时操作，每累加一次耗时0.1s，
        为了良好的用户体验，需要服务器在计算过程中，
        每累加一次，就给客户端响应一次百分比格式的执行进度，使用action实现。

    流程:
        1.包含头文件;
        2.初始化ROS节点;
        3.创建NodeHandle;
        4.创建action服务对象;
        5.处理请求,产生反馈与响应;
        6.spin().

*/

typedef actionlib::SimpleActionServer<pkg_nav::AddIntsAction> Server;

void cb(const pkg_nav::AddIntsGoalConstPtr &goal, Server *server)
{
    //获取目标值
    int num = goal->num;
    ROS_INFO("目标值:%d", num);
    //累加并响应连续反馈
    int result = 0;
    pkg_nav::AddIntsFeedback feedback; //连续反馈
    ros::Rate rate(1);                 //通过频率设置休眠时间
    for (int i = 1; i <= num; i++)
    {
        result += i;
        //组织连续数据并发布
        feedback.progress_bar = i / (double)num;
        ROS_INFO("最终结果:%.2f", 1.0 * i / num);
        server->publishFeedback(feedback);
        rate.sleep();
    }
    //设置最终结果
    pkg_nav::AddIntsResult r;
    r.result = result;
    server->setSucceeded(r);
    ROS_INFO("最终结果:%d", r.result);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ROS_INFO("action服务端实现");
    // 2.初始化ROS节点;
    ros::init(argc, argv, "AddInts_server");
    // 3.创建NodeHandle;
    ros::NodeHandle nh;
    // 4.创建action服务对象;
    /*SimpleActionServer(ros::NodeHandle n, 
                        std::string name, 
                        boost::function<void (const pkg_nav::AddIntsGoalConstPtr &)> execute_callback, 
                        bool auto_start)
    */
    // actionlib::SimpleActionServer<pkg_nav::AddIntsAction> server(....);
    Server server(nh, "addInts", boost::bind(&cb, _1, &server), false);
    server.start();
    // 5.处理请求,产生反馈与响应;

    // 6.spin().
    ros::spin();
    return 0;
}