#include "ros/ros.h"
#include "actionlib/server/simple_action_server.h"
#include "pkg_nav/local_pathAction.h"

typedef actionlib::SimpleActionServer<pkg_nav::local_pathAction> Server;

void cb(const pkg_nav::local_pathGoalConstPtr &goal, Server *server)
{
    //获取目标值
    int num = goal->num;
    ROS_INFO("目标值:%d", num);
    //累加并响应连续反馈
    int result = 0;
    pkg_nav::local_pathFeedback feedback; //连续反馈
    ros::Rate rate(1);                    //通过频率设置休眠时间
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
    pkg_nav::local_pathResult r;
    r.result = result;
    server->setSucceeded(r);
    ROS_INFO("最终结果:%d", r.result);
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    ROS_INFO("action服务端实现");
    // 2.初始化ROS节点;
    ros::init(argc, argv, "local_path_server");
    // 3.创建NodeHandle;
    ros::NodeHandle nh;
    // 4.创建action服务对象;
    /*SimpleActionServer(ros::NodeHandle n, 
                        std::string name, 
                        boost::function<void (const pkg_nav::local_pathGoalConstPtr &)> execute_callback, 
                        bool auto_start)
    */
    // actionlib::SimpleActionServer<pkg_nav::local_pathAction> server(....);
    Server server(nh, "local_path", boost::bind(&cb, _1, &server), false);
    server.start();
    // 5.处理请求,产生反馈与响应;

    // 6.spin().
    ros::spin();
    return 0;
}