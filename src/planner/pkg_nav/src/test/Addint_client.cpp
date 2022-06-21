#include "ros/ros.h"
#include "actionlib/client/simple_action_client.h"
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
        4.创建action客户端对象;
        5.发送目标，处理反馈以及最终结果;
        6.spin().

*/
typedef actionlib::SimpleActionClient<pkg_nav::AddIntsAction> Client;

//处理最终结果
void done_cb(const actionlib::SimpleClientGoalState &state, const pkg_nav::AddIntsResultConstPtr &result)
{
    if (state.state_ == state.SUCCEEDED)
    {
        ROS_INFO("最终结果:%d", result->result);
    }
    else
    {
        ROS_INFO("任务失败！");
    }
}
//服务已经激活
void active_cb()
{
    ROS_INFO("服务已经被激活....");
}
//处理连续反馈
void feedback_cb(const pkg_nav::AddIntsFeedbackConstPtr &feedback)
{
    ROS_INFO("当前进度:%.2f", feedback->progress_bar);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    // 2.初始化ROS节点;
    ros::init(argc, argv, "AddInts_client");
    // 3.创建NodeHandle;
    ros::NodeHandle nh;
    // 4.创建action客户端对象;
    // SimpleActionClient(ros::NodeHandle & n, const std::string & name, bool spin_thread = true)
    // actionlib::SimpleActionClient<pkg_nav::AddIntsAction> client(nh,"addInts");
    Client client(nh, "addInts", true);
    //等待服务启动
    client.waitForServer();
    // 5.发送目标，处理反馈以及最终结果;
    /*  
        void sendGoal(const pkg_nav::AddIntsGoal &goal, 
            boost::function<void (const actionlib::SimpleClientGoalState &state, const pkg_nav::AddIntsResultConstPtr &result)> done_cb, 
            boost::function<void ()> active_cb, 
            boost::function<void (const pkg_nav::AddIntsFeedbackConstPtr &feedback)> feedback_cb)

    */
    pkg_nav::AddIntsGoal goal;
    goal.num = 10;

    client.sendGoal(goal, &done_cb, &active_cb, &feedback_cb);
    // 6.spin().
    ros::spin();
    return 0;
}