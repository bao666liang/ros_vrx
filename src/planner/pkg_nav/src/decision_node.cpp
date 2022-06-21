//Date: 2021 12-10-10:00
#pragma region //include  #pragma endregion
#include<pkg_nav/decision_node.h>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "pkg_nav/local_pathAction.h"
// #include "pkg_nav/global_pathAction.h"
#include "pkg_nav/global_path.h"
#include <boost/thread.hpp>

#include <visualization_msgs/MarkerArray.h>
#include "pkg_nav//global_paln/a_star_search.h"
#include "pkg_nav/osqp_problem.h"
#include "pkg_nav/smoothosqpproblem.h"
#include "pkg_nav/system_command.h"

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <sstream>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <rosbag/bag.h>
#include <ros/package.h>
#include <time.h>
#include <ctime>
#include <pkg_nav/utility.h>
#include <pkg_nav/local_paln/local_paln_offset.h>

using namespace std;
using namespace Eigen;
// global_pathAction;
typedef actionlib::SimpleActionClient<pkg_nav::local_pathAction> Client;

#pragma endregion //include  #pragma endregion

class Decision
{
#pragma region //Decision  #pragma endregion
private:
    ros::NodeHandle m_nh;
    ros::NodeHandle m_Private_nh;

    ros::Subscriber m_global_map_sub; //Subscriber global map
    ros::Subscriber m_local_map_sub;  //Subscriber local map
    ros::Subscriber m_odometry_sub;   //Subscriber local map
    ros::Subscriber m_global_start_pose_sub;
    ros::Subscriber m_global_goal_pose_sub;
    ros::Subscriber m_local_start_pose_sub;
    ros::Subscriber m_local_goal_pose_sub;
    ros::Publisher m_global_path_pub;       //Publisher global path
    ros::Publisher m_local_path_pub;        //Publisher local path
    ros::Publisher m_global_start_pose_pub; //Publisher global start pose
    ros::Publisher m_global_goal_pose_pub;  //Publisher global goal pose
    ros::Publisher mMarkerPub = m_nh.advertise<visualization_msgs::Marker>("Robot_marker", 1);
    uint32_t mShape = visualization_msgs::Marker::SPHERE; //SPHERE CUBE
    visualization_msgs::Marker mMarker;
    nav_msgs::Path m_global_path;
    nav_msgs::Path m_local_path;
    nav_msgs::Odometry mOdometry;
    ros::Publisher mOdometryPub;

    Client m_local_path_client; //client(nh, "/ow/local_path", true);
    ros::ServiceServer m_global_server;
    ros::ServiceClient m_global_client;

    // costmap as occupancy grid
    nav_msgs::OccupancyGrid m_global_map;
    int m_global_map_height;
    int m_global_map_width;
    float m_global_map_resolution;
    nav_msgs::OccupancyGrid m_local_map;
    int m_local_map_height;
    int m_local_map_width;
    float m_local_map_resolution;
    nav_msgs::Odometry m_odometry;

    // thread
    boost::mutex m_global_path_mutex;
    boost::thread *m_global_plan_thread;
    boost::thread *m_local_plan_thread;
    geometry_msgs::PoseStamped m_global_start_pose;
    geometry_msgs::PoseStamped m_global_goal_pose;
    geometry_msgs::PoseStamped m_local_start_pose;
    geometry_msgs::PoseStamped m_local_goal_pose;
    std::vector<geometry_msgs::PoseStamped> m_global_goal_pose_vec;

    bool m_global_map_initialized;
    bool m_local_map_initialized;
    bool m_global_path_initialized;
    bool m_local_path_initialized;
    bool m_global_start_pose_initialized;
    bool m_global_goal_pose_initialized;
    bool m_local_start_pose_initialized;
    bool m_local_goal_pose_initialized;
    size_t m_local_goal_pose_index = 0;

    E_SystemRunState::E_SystemRunState m_run_state;
    E_SystemErrorState::E_SystemErrorState m_err_state;

    ros::Publisher m_global_raw_path_pub;
    ros::Publisher m_act_path_pub;
    ros::Publisher m_local_raw_path_pub;
    ros::Publisher m_system_command_pub; //系统控制命令
    nav_msgs::Path m_global_raw_path;
    nav_msgs::Path m_act_path;
    nav_msgs::Path m_local_raw_path;
    pkg_nav::system_command m_system_command;
    E_PathShape::E_PathShape mPathShae = E_PathShape::AStar;

    geometry_msgs::PoseStamped mstartPoseTest;//test
    double mSetDistLocalStartToGoal;     //set distance
    double mActDistLocalStartToGoal;     //act distance
    int mIndexOffsetLocalGoalPose;
    int mTestLine = 0;
    unsigned mIndex = 0;

    ofstream mOutFile;
    ostringstream m_cwdss;
#pragma endregion

#pragma region //initialize #pragma endregion
public:
    Decision(const std::string client_name, bool flag = true) : m_local_path_client(client_name, flag)
    {
        ROS_INFO("The Decision::Decision() fun is executed!");
        m_run_state = E_SystemRunState::IN_idle;
        ros::Rate cycle_rate(1);
        size_t count = 1;
        while (ros::ok())
        {
            switch (m_run_state)
            {
            case E_SystemRunState::IN_idle:
                ROS_INFO("系统处于闲置状态！");
                m_run_state = E_SystemRunState::IN_initializing;
                break;

            case E_SystemRunState::IN_initializing:
                ROS_INFO("系统正在初始化...");
                initialize();
                m_run_state = E_SystemRunState::IN_waiting_command;

            case E_SystemRunState::IN_waiting_command:
                ROS_INFO("系统正在等待命令...");
                m_run_state = E_SystemRunState::IN_planning;
                break;

            case E_SystemRunState::IN_planning:

                ROS_INFO("系统正在进行规划和控制...");
                
                // m_run_state = E_SystemRunState::IN_normal_stop;
                break;

            case E_SystemRunState::IN_normal_stop:
                ROS_INFO("系统正在进行正常停止...");
                m_run_state = E_SystemRunState::IN_emergency_stop;
                break;

            case E_SystemRunState::IN_emergency_stop:
                ROS_INFO("系统正在进行急停...");
                m_run_state = E_SystemRunState::IN_reset;
                break;

            case E_SystemRunState::IN_reset:
                ROS_INFO("系统正在进行复位...");
                Reset();
                m_run_state = E_SystemRunState::IN_idle;
                break;

            default:
                ROS_INFO("系统处于默认初始状态...");
                printf("%d = ", m_run_state);
                break;
            }
            cycle_rate.sleep();
            count++;
        }
    }

    void initialize()
    {
        // costmap as occupancy grid
        m_global_map.data.clear();
        m_global_map_height = 0;
        m_global_map_width = 0;
        m_global_map_resolution = 0.0;
        m_local_map.data.clear();
        m_local_map_height = 0;
        m_local_map_width = 0;
        m_local_map_resolution = 0.0;

        // thread
        m_global_plan_thread = nullptr;
        m_local_plan_thread = nullptr;
        //  m_global_start_pose.;
        //  m_global_goal_pose;
        // m_local_start_pose;
        // m_local_goal_pose;

        m_global_map_initialized = false;
        m_local_map_initialized == false;
        m_global_path_initialized = false;
        m_local_path_initialized = false;
        m_global_start_pose_initialized = false;
        m_global_goal_pose_initialized = false;
        m_local_start_pose_initialized = false;
        m_local_goal_pose_initialized = false;
        m_err_state = E_SystemErrorState::Err_ok;

        int value = 0;
        m_Private_nh.param("pathShape", value, 0);
        ROS_ERROR("The value = %d", value);
        if (value >= 0 && value <= 6)
        {
            mPathShae = (E_PathShape::E_PathShape)value;
            m_Private_nh.param("distCheckLocalStartToGoal", mSetDistLocalStartToGoal, 60.0); //10m
            m_Private_nh.param("indexOffsetLocalGoalPose", mIndexOffsetLocalGoalPose, 100);  //mTestLine
            m_Private_nh.param("isTestLine", mTestLine, 0);                                   //mTestLine
            m_local_goal_pose_index = 100;
        }
        else
        {
            mPathShae = E_PathShape::AStar;
            m_local_goal_pose_index = 100;
        }
        m_global_plan_thread = new boost::thread(boost::bind(&Decision::global_plan_thread, this));
        m_local_plan_thread = new boost::thread(boost::bind(&Decision::local_plan_thread, this));
        // 基于当前系统的当前日期/时间
        time_t now = time(0); //now = 1638537838
        // 把 now 转换为字符串形式
        char *dt = ctime(&now); //Fri Dec  3 21:34:50 2021
        dt[strlen(dt) - 1] = 0;
        string str = dt;
        np::erase_all_space(str);
        m_cwdss << "act_path_" << str << ".csv";
    }

    void Reset()
    {
        delete m_global_plan_thread;
        delete m_local_plan_thread;
        m_global_plan_thread = nullptr;
        m_local_plan_thread = nullptr;
    }

    void global_map_cb(const nav_msgs::OccupancyGrid::ConstPtr &global_map)
    {
        m_global_map_initialized = false;
        m_global_map_height = global_map->info.height;
        m_global_map_width = global_map->info.width;
        m_global_map_resolution = global_map->info.resolution;
        //  global_map->header.frame_id  //TODO
        //  global_map->header.stamp  //TODO
        int len = global_map->data.size();
        if (len == m_global_map_height * m_global_map_width)
        {
            m_global_map_initialized = true;
            // return;
            m_global_map = *global_map;
            ROS_INFO("The global_map 高*宽 = %d*%d", m_global_map_height, m_global_map_width);
        }
        else
        {
            ROS_FATAL("The global_map 大小(%d) != 高(%d)*宽(%d)", len, m_global_map_height, m_global_map_width);
        }
    }

    void local_map_cb(const nav_msgs::OccupancyGrid::ConstPtr &local_map)
    {
        m_local_map_initialized = false;
        m_local_map_height = local_map->info.height;
        m_local_map_width = local_map->info.width;
        m_local_map_resolution = local_map->info.resolution;
        // m_local_map_resolution = local_map->header.frame_id  //TODO
        // m_local_map_resolution = local_map->header.stamp  //TODO
        int len = local_map->data.size();
        if (len == m_local_map_height * m_local_map_width)
        {
            m_local_map_initialized = true;
            m_local_map = *local_map;
            return;
            ROS_INFO("The local_map 高*宽 = %d*%d", m_local_map_height, m_local_map_width);
        }
        else
        {
            ROS_WARN("The local_map 大小(%d) != 高(%d)*宽(%d)", len, m_local_map_height, m_local_map_width);
        }

        ;
    }

    void odometry_cb(const nav_msgs::Odometry odometry_msg)
    {
        geometry_msgs::PoseStamped tempPose;
        tempPose.pose = odometry_msg.pose.pose;
        m_act_path.poses.push_back(tempPose);
        m_act_path.header.frame_id = "map";
        m_act_path_pub.publish(m_act_path);
        m_odometry = odometry_msg;
        m_global_start_pose.header.frame_id = odometry_msg.header.frame_id;
        m_global_start_pose.pose = odometry_msg.pose.pose;
        m_global_start_pose_initialized = true;
        m_local_start_pose.header.frame_id = odometry_msg.header.frame_id;
        m_local_start_pose.pose = odometry_msg.pose.pose;
        m_local_start_pose_initialized = true;
       
        mOutFile.open(m_cwdss.str(), ios::app); // 打开模式可省略
        if (!mOutFile.is_open())
        {
            ROS_FATAL("Open file failure");
        }
        // ostringstream xyss;
        // xyss << "x_act" << ","<< "y_act";
        // mOutFile << xyss.str() << std::endl;
        mOutFile << tempPose.pose.position.x << "," <<  tempPose.pose.position.y << std::endl;
        mOutFile.close();
    }

#pragma endregion

#pragma region //lineMotionGlobal
    bool lineMotionGlobal()
    {
        if (!m_global_path_initialized)
        {
            mstartPoseTest = m_global_start_pose;
        }
        if (mPathShae == E_PathShape::Square)
        {
            m_global_goal_pose_vec.clear();
            geometry_msgs::PoseStamped tempPose;
            tempPose.pose.position.x = m_global_goal_pose.pose.position.x;
            tempPose.pose.position.y = m_global_goal_pose.pose.position.y;
            m_global_goal_pose_vec.push_back(tempPose);
            tempPose.pose.position.x += 100;
            tempPose.pose.position.y += 0;
            m_global_goal_pose_vec.push_back(tempPose);
            tempPose.pose.position.x += 0;
            tempPose.pose.position.y -= 100;
            m_global_goal_pose_vec.push_back(tempPose);
            tempPose.pose.position.x -= 100;
            tempPose.pose.position.y += 0;
            m_global_goal_pose_vec.push_back(tempPose);
            tempPose.pose.position.x += 0;
            tempPose.pose.position.y += 100;
            m_global_goal_pose_vec.push_back(tempPose);
        }
        nav_msgs::Path path;
        if (mPathShae == E_PathShape::Line)
        {
            float dx = m_global_goal_pose.pose.position.x - mstartPoseTest.pose.position.x;
            float dy = m_global_goal_pose.pose.position.y - mstartPoseTest.pose.position.y;
            int countPath = sqrt(dx * dx + dy * dy);
            geometry_msgs::PoseStamped tempPose;
            for (size_t j = 0; j < countPath; j++)
            {
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + j * dx / countPath;
                tempPose.pose.position.y = mstartPoseTest.pose.position.y + j * dy / countPath;
                path.poses.push_back(tempPose);
            }
            path.poses.push_back(m_global_goal_pose);
            ROS_WARN("The lineMotion countPath = %d", countPath);
            m_global_path = path;
            return true;
        }
        else if (mPathShae == E_PathShape::Square)
        {
            int indexGoalPos = 0;
            // float dx = m_global_goal_pose.pose.position.x - mstartPoseTest.pose.position.x;
            // float dy = m_global_goal_pose.pose.position.y - mstartPoseTest.pose.position.y;
            // int countPath = sqrt(dx * dx + dy * dy);
            // geometry_msgs::PoseStamped tempPose;
            // for (size_t j = 0; j < countPath; j++)
            // {
            //     tempPose.pose.position.x = mstartPoseTest.pose.position.x + j * dx / countPath;
            //     tempPose.pose.position.y = mstartPoseTest.pose.position.y + j * dy / countPath;
            //     path.poses.push_back(tempPose);
            // }
            // path.poses.push_back(m_global_goal_pose_vec[indexGoalPos]);

            for (size_t i = indexGoalPos; i < m_global_goal_pose_vec.size() - 1; i++)
            {
                float dx = m_global_goal_pose_vec[i + 1].pose.position.x - m_global_goal_pose_vec[i].pose.position.x;
                float dy = m_global_goal_pose_vec[i + 1].pose.position.y - m_global_goal_pose_vec[i].pose.position.y;
                int countPath = sqrt(dx * dx + dy * dy);
                geometry_msgs::PoseStamped tempPose;
                for (size_t j = 0; j < countPath; j++)
                {
                    tempPose.pose.position.x = m_global_goal_pose_vec[i].pose.position.x + j * dx / countPath;
                    tempPose.pose.position.y = m_global_goal_pose_vec[i].pose.position.y + j * dy / countPath;
                    path.poses.push_back(tempPose);
                }
                path.poses.push_back(m_global_goal_pose_vec[i + 1]);
            }
        }
        m_global_path = path;
        return true;
    }

    bool CircleMotionGlobal()
    {
        nav_msgs::Path path;
        int count = 360 / 1;  //360°/resolution
        std::vector<std::pair<float, float>> xy;
        if (!m_global_path_initialized)
        {
            mstartPoseTest = m_global_start_pose;
        }
        float dx = m_global_goal_pose.pose.position.x - mstartPoseTest.pose.position.x;
        float dy = m_global_goal_pose.pose.position.y - mstartPoseTest.pose.position.y;
        float r = sqrt(dx * dx + dy * dy)/2;
        
        geometry_msgs::PoseStamped tempPose;
        if (mPathShae == E_PathShape::CircleS)
        {
            for (size_t i = 0; i < count/2; i++)
            {
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + r * cos(M_PI - i * M_PI / 180) + r;
                tempPose.pose.position.y = mstartPoseTest.pose.position.y + r * sin(M_PI - i * M_PI / 180);
                path.poses.push_back(tempPose);
            }
            for (size_t i = count / 2; i <= count; i++)
            {
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + r * cos(i * M_PI / 180) + 3 * r;
                tempPose.pose.position.y = mstartPoseTest.pose.position.y + r * sin(i * M_PI / 180);
                path.poses.push_back(tempPose);
            }
            m_global_path = path;
            ROS_WARN("CircleS = true");
            return true;
        }
        if (mPathShae == E_PathShape::SpiralLine)
        {
            for (size_t i = 0; i < count*10; i++)
            {
                r = 1.0 * i / count * 10;
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + r * cos(i * M_PI / 180);
                tempPose.pose.position.y = mstartPoseTest.pose.position.y + r * sin(i * M_PI / 180);
                path.poses.push_back(tempPose);
            }
            m_global_path = path;
            ROS_WARN("SpiralLine = true");
            return true;
        }

        for (size_t i = 0; i < count; i++) //Circle
        {
            tempPose.pose.position.x = mstartPoseTest.pose.position.x + r * cos(i * M_PI / 180) - r;
            tempPose.pose.position.y = mstartPoseTest.pose.position.y + r * sin(i * M_PI / 180);
            path.poses.push_back(tempPose);
        }

        if(mPathShae == E_PathShape::CircleDouble)
        {
            for (size_t i = 0; i < count; i++)
            {
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + r * cos(M_PI - i * M_PI / 180) + r;
                tempPose.pose.position.y = mstartPoseTest.pose.position.y + r * sin(M_PI - i * M_PI / 180);
                path.poses.push_back(tempPose);
            }
        }
        m_global_path = path;
        return true;
    }

    bool WriteFile(string pathStr, const nav_msgs::Path &path,size_t count) // 写文件
    {
        ofstream outFile;
        ostringstream cwdss;
        cwdss << pathStr<<".csv";
        outFile.open(cwdss.str(), ios::app); // 打开模式可省略
        if (!outFile.is_open())
        {
            ROS_FATAL("Open file failure");
        }
        else
        {
            ostringstream xss;
            ostringstream yss;
            xss << "x" << count << ",";
            yss << "y" << count << ",";
            for (auto pose : path.poses)
            {
                xss << pose.pose.position.x << ",";
                yss << pose.pose.position.y << ",";
            }
            outFile << xss.str() << std::endl;
            outFile << yss.str() << std::endl;
        }
        outFile.close();
        return true;
    }
#pragma endregion

    void global_plan_thread()
    {
        ROS_INFO("The Decision::global_plan_thread 被执行 ...");
        m_global_raw_path_pub = m_nh.advertise<nav_msgs::Path>("globa_raw_path", 1);
        m_global_path_pub = m_nh.advertise<nav_msgs::Path>("/ow/global_path", 1);
        m_act_path_pub = m_nh.advertise<nav_msgs::Path>("/ow/act_path", 1);
        m_global_goal_pose_pub = m_nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", 1);
        // m_global_start_pose_pub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);

        m_global_map_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>("/ow/global_map", 1, &Decision::global_map_cb, this);
        m_odometry_sub = m_nh.subscribe<nav_msgs::Odometry>("/ow/odometry", 1, &Decision::odometry_cb, this);
        m_global_start_pose_sub = m_nh.subscribe("/initialpose", 1, &Decision::global_start_pose_cb, this);
        m_global_goal_pose_sub = m_nh.subscribe("move_base_simple/goal", 1, &Decision::global_goal_pose_cb, this);
        // m_global_server = m_nh.advertiseService("/ow/global_path", &Decision::global_plan_response, this);
        // ros::service::waitForService("/ow/global_path");
        // ROS_WARN_THROTTLE(3, "The global_server 服务已经启动，请设置全局地图及起始点....");
        // m_global_client = m_nh.serviceClient<pkg_nav::global_path>("/ow/global_path");

        pkg_nav::global_path global_path_request;
        unsigned count = 1;
        ros::Rate loop_rate(0.1);
        // 写文件
        // 基于当前系统的当前日期/时间
        time_t now = time(0); //now = 1638537838
        // 把 now 转换为字符串形式
        char *dt = ctime(&now); //Fri Dec  3 21:34:50 2021
        dt[strlen(dt) - 1] = 0;
        string fileName = dt;
        np::erase_all_space(fileName);
        fileName = "global_path_" + fileName;
        while (ros::ok)
        {
            if (m_run_state != E_SystemRunState::IN_planning)
            {
                loop_rate.sleep();
                continue;
            }
            ros::spinOnce();
            if (global_path_update())
            {
                global_path_request.request.num = 10;
                global_path_request.request.start_pose = m_global_start_pose;
                global_path_request.request.goal_pose = m_global_goal_pose;
               
                // ros::service::waitForService("/ow/global_path");
                // ROS_INFO("The m_global_path_pub count === %d", count++);
                // m_global_path_initialized = m_global_client.call(global_path_request);
                if (mPathShae == E_PathShape::AStar) //A*
                {
                    ROS_WARN("global_plan_response = true");
                    if (mActDistLocalStartToGoal > mSetDistLocalStartToGoal)
                    {
                        m_global_path_initialized = global_plan_response(global_path_request.request, global_path_request.response);
                        // m_global_start_pose_initialized = false;
                    }                
                }
                else if (mPathShae == E_PathShape::Line || mPathShae == E_PathShape::Square) //line // //square
                {
                    ROS_WARN("lineMotionGlobal = true");
                    m_global_path_initialized = lineMotionGlobal();
                }
                else if (mPathShae == E_PathShape::Circle || mPathShae == E_PathShape::CircleDouble || mPathShae == E_PathShape::CircleS || mPathShae == E_PathShape::SpiralLine) //circle
                {
                    ROS_WARN("CircleMotionGlobal = true");
                    m_global_path_initialized = CircleMotionGlobal();
                }
                if (m_global_path_initialized)
                {
                    boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
                    if (mPathShae == E_PathShape::AStar)
                    {
                        m_global_path = global_path_request.response.global_path; //TODO
                    }
                    // mIndex = 0;
                    m_global_path.header.frame_id = "map"; //TODO
                    m_global_path.header.seq = count;
                    m_global_path.header.stamp = ros::Time::now();
                    m_global_raw_path.header.frame_id = "map";
                   
                    m_global_raw_path_pub.publish(m_global_raw_path);
                    m_global_path_pub.publish(m_global_path);
                   
                    // ROS_INFO("The m_global_client 已经获取到 global_path ：sum = %d", global_path_request.response.sum);
                    WriteFile(fileName, m_global_path, count); //写文件
                    ROS_WARN("The m_global_path_pub count = %d", count++);

                    // m_global_path_initialized = false;
                    // m_global_start_pose_initialized = false;
                    // m_global_map_initialized = false;
                }
                else
                {
                    ROS_FATAL("The global_server 执行失败....");
                    m_err_state = E_SystemErrorState::Err_global_planning;
                    m_run_state = E_SystemRunState::IN_emergency_stop;
                    m_global_path.poses.clear();
                    m_global_path_pub.publish(m_global_path);
                    return;
                }
            }
            loop_rate.sleep();
        }
    }

    // bool 返回值用于标志是否处理成功
    bool global_plan_response(pkg_nav::global_path::Request &request, pkg_nav::global_path::Response &response)
    {
        int num = request.num;
        int sum = 0;
        ROS_INFO("The m_global_server 正在处理数据... ,request.num = %d", num);
        if (num < 0)
        {
            ROS_FATAL("提交的数据异常：数据不可以为负数 ！");
            return false;
        }

        ros::NodeHandle nh;
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("a_star_path", 1);
        ros::Publisher circle_pub = nh.advertise<visualization_msgs::MarkerArray>("a_star_circle", 1);

        geometry_msgs::Pose start_pose, goal_pose;
        bool is_point_robot = true; // 是否为点状机器人
        // 可视化非常耗费时间, 若不需要查看A*算法中的探索过程,
        // 建议将可视化的标志位置为false
        bool is_visualization = false;  // 是否需要可视化中间过程
        bool is_eight_connected = true; // 每个节点的方向是否为八连通 //TODO
        std::unique_ptr<AStar> a_star_ptr = std::make_unique<AStar>(nh, is_visualization, is_eight_connected);
        AStarResult a_star_result;
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        a_star_ptr->SetMap(m_global_map);
        if (!is_point_robot)
        {
            // 如果不是点机器人, 则需要设置长宽等参数
            a_star_ptr->SetVhicleParams(5.0, 5.0, 3.0);
        }
        ros::Time start_time = ros::Time::now();
        start_pose = request.start_pose.pose;
        goal_pose = request.goal_pose.pose;
        int heuristic = 2;
        if (a_star_ptr->SearchPath(start_pose.position.x, start_pose.position.y, goal_pose.position.x, goal_pose.position.y, &a_star_result, heuristic))
        {
            float delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[查找global_path  delta_time =  " << delta_time << " s]!" << std::endl;
            size_t len = a_star_result.x.size();
            geometry_msgs::PoseStamped pose;
            int path_smooth_methed = 2;
            if (0 == path_smooth_methed) ////raw path
            {
                for (size_t i = 0; i < len; i++)
                {
                    pose.pose.position.x = a_star_result.x[i];
                    pose.pose.position.y = a_star_result.y[i];
                    path.poses.push_back(std::move(pose));
                }
            }
            else if (1 == path_smooth_methed) // //cubic Bezuer_curve
            {
                nav_msgs::Path raw_path;
                geometry_msgs::PoseStamped raw_pose;
                for (int i = 0; i < len; ++i)
                {
                    raw_pose.pose.position.x = a_star_result.x[i];
                    raw_pose.pose.position.y = a_star_result.y[i];
                    raw_path.poses.push_back(raw_pose);
                }
                m_global_raw_path = raw_path;

                std::vector<geometry_msgs::Point> raw_point3d;
                std::vector<geometry_msgs::Point> smooth_point3d;
                geometry_msgs::Point point;
                for (size_t i = 0; i < len; i += 10)
                {
                    point.x = a_star_result.x[i];
                    point.y = a_star_result.y[i];
                    raw_point3d.push_back(point);
                }
                if (len % 10 != 1)
                {
                    point.x = a_star_result.x[len - 1];
                    point.y = a_star_result.y[len - 1];
                    raw_point3d.push_back(point);
                }
                Bezuer::Bezuer_curve Bezuer_curve(raw_point3d, smooth_point3d, 10);
                for (size_t i = 0; i < len; i++)
                {
                    pose.pose.position = smooth_point3d[i];
                    path.poses.push_back(std::move(pose));
                }
            }
            else if (2 == path_smooth_methed) // //quadratic programming
            {
                std::vector<std::pair<double, double>> raw_point2d;
                std::vector<std::pair<double, double>> upperBounds;
                std::vector<std::pair<double, double>> lowerBounds;
                std::vector<double> bounds;
                geometry_msgs::PoseStamped raw_pose;
                nav_msgs::Path raw_path;
                double x, y;
                int distBound = 20;
                double boundLU = distBound / 3;
                int distPerPixel = 1 / m_global_map.info.resolution;
                for (int i = 0; i < len; i++)
                {
                    bool bXLeft = true;
                    bool bXRight = true;
                    bool bYDown = true;
                    bool bYUp = true;
                    bool bXYLeftDown = true;
                    bool bXYLeftUp = true;
                    bool bXYRightDown = true;
                    bool bXYRightUp = true;
                    int xBound = 0;
                    int yBound = 0;
                    int xyBound = 0;
                    int xFlag = 0;
                    int yFlag = 0;

                    int index_x = a_star_result.x[i]/ m_global_map.info.resolution;
                    int index_y = a_star_result.y[i] / m_global_map.info.resolution;
                    const int index = CalcIndex(index_x, index_y, m_global_map);           // 计算在一维数组中的索引
                   
                    //========== x bound =======================
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //x left
                    {
                        if (m_global_map.data[index - j] != 0)
                        {
                            bXLeft = false;
                            break;
                        }
                    }
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //x right
                    {
                        if (m_global_map.data[index + j] != 0)
                        {
                            bXRight = false;
                            break;
                        }
                    }
                    if (bXLeft  && !bXRight )
                    {
                        xBound = -distBound / 2 ;
                    }
                    else if (!bXLeft && bXRight)
                    {
                        xBound= distBound / 2;
                    }
                    else
                    {
                        xBound= 0;
                    }

                    //========== y bound =======================
                    if (xBound != 0)
                    {
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y down
                        {
                            if (m_global_map.data[index + xBound / m_global_map.info.resolution - j * m_global_map.info.width] != 0)
                            {
                                bYDown = false;
                                break;
                            }
                        }
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y up
                        {
                            if (m_global_map.data[index + xBound / m_global_map.info.resolution +  j * m_global_map.info.width] != 0)
                            {
                                bYUp = false;
                                break;
                            }
                        }
                        if (bYDown && !bYUp)
                        {
                            yBound = -distBound / 2;
                        }
                        else if (!bYDown && bYUp)
                        {
                            yBound = distBound / 2;
                        }
                        else
                        {
                            yBound = 0;
                        }
                    }
                    else
                    {
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y down
                        {
                            if (m_global_map.data[index - j * m_global_map.info.width] != 0)
                            {
                                bYDown = false;
                                break;
                            }
                        }
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y up
                        {
                            if (m_global_map.data[index + j * m_global_map.info.width] != 0)
                            {
                                bYUp = false;
                                break;
                            }
                        }
                        if (bYDown && !bYUp)
                        {
                            yBound = -distBound / 2;
                        }
                        else if (!bYDown && bYUp)
                        {
                            yBound = distBound / 2;
                        }
                        else
                        {
                            yBound = 0;
                        }
                    }

                    //========== -x -y <--> +x +y bound =======================
                    if (xBound == 0 &&  yBound == 0)
                    {
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x left
                        {
                            if (m_global_map.data[index - j - j * m_global_map.info.width] != 0)
                            {
                                bXYLeftDown = false;
                                break;
                            }
                        }
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x right
                        {
                            if (m_global_map.data[index + j + j * m_global_map.info.width] != 0)
                            {
                                bXYRightUp = false;
                                break;
                            }
                        }
                        if (bXYLeftDown && !bXYRightUp)
                        {
                            xBound = -distBound / 2;
                            yBound = -distBound / 2;
                        }
                        if (bXYRightUp && !bXYLeftDown)
                        {
                            xBound = distBound / 2;
                            yBound = distBound / 2;
                            ROS_FATAL("!bXYLeftDown && bXYRightUp");
                        }

                        //========== -x +y <--> +x -y bound =======================
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x left
                        {
                            if (m_global_map.data[index - j + j * m_global_map.info.width] != 0)
                            {
                                bXYLeftUp = false;
                                break;
                            }
                        }
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x right
                        {
                            if (m_global_map.data[index + j - j * m_global_map.info.width] != 0)
                            {
                                bXYRightDown = false;
                                break;
                            }
                        }
                        if (bXYLeftUp && !bXYRightDown)
                        {
                            xBound = -distBound / 2.0;
                            yBound = distBound / 2.0;
                        }
                        else if (!bXYLeftUp && bXYRightDown)
                        {
                            xBound = distBound / 2.0;
                            yBound = -distBound / 2.0;
                        }
                    }
                    
                    x = a_star_result.x[i] + xBound * 1 ;
                    y = a_star_result.y[i] + yBound * 1 ;
                    raw_pose.pose.position.x = a_star_result.x[i];
                    raw_pose.pose.position.y = a_star_result.y[i] ;
                    raw_path.poses.push_back(raw_pose);
                    raw_point2d.push_back(std::make_pair(x, y));
                    // raw_point2d.push_back(std::make_pair(a_star_result.x[i], a_star_result.y[i]));
                    upperBounds.push_back(std::make_pair(boundLU, boundLU));
                    lowerBounds.push_back(std::make_pair(boundLU, boundLU));
                }
                // std::vector<std::pair<double, double>> ref_point2d;
                // PathFitter(raw_point2d, ref_point2d);
                // for (size_t i = 0; i < ref_point2d.size(); i++)
                // {
                //     upperBounds.push_back(std::make_pair(5, 5));
                //     lowerBounds.push_back(std::make_pair(5, 5));
                // }
                upperBounds[0].first = 0.1;
                upperBounds[0].second = 0.1;
                upperBounds[upperBounds.size() - 1].first = 0.1;
                upperBounds[upperBounds.size() - 1].second = 0.1;
                lowerBounds[0].first = 0.1;
                lowerBounds[0].second = 0.1;
                lowerBounds[lowerBounds.size() - 1].first = 0.1;
                lowerBounds[lowerBounds.size() - 1].second = 0.1;
                m_global_raw_path = raw_path;
                FemPosDeviationSqpOsqpInterface solver;
                std::cout << "OSQP RUNNING..." << std::endl;
                solver.set_ref_points(raw_point2d);
                solver.set_bounds_around_refs(upperBounds, lowerBounds);
                if (!solver.Solve())
                {
                    std::cout << "failture" << std::endl;
                    ROS_FATAL("The FemPosDeviationSqpOsqpInterface solver is  failture !");
                }
                std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();
                std::cout << "OSQP END!!!" << std::endl;
                for (size_t i = 0; i < opt_xy.size(); i++)
                {
                    pose.pose.position.x = opt_xy[i].first;
                    pose.pose.position.y = opt_xy[i].second;
                    path.poses.push_back(std::move(pose));
                }
            }
            delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[路径处理总时间 global_path  delta_time =  " << delta_time << " s]!" << std::endl;
        }
        else
        {
            ROS_FATAL("全局路径查找失败 ！");
            return false;
        }

        for (int i = 1; i <= num; i++)
        {
            sum += i;
        }
        response.sum = sum;
        response.global_path = path;
        ROS_WARN("The test for global_plan_response of  global_path is normal");
        return true;
    }

#pragma region //#get_local_path
    bool get_local_path()
    {
        boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
        int len = m_global_path.poses.size();
        std::cout << "The m_global_path len = " << len << std::endl;
        if (len < 1)
        {
            return false;
        }
        // if (np::caculate_distance(m_global_path.poses[len - 1].pose, m_local_start_pose.pose) < 100)
        if (len <= mIndexOffsetLocalGoalPose)
        {
            std::cout << "The  m_global_path.poses[" << len - 1 << "] = " << m_global_path.poses[len - 1] << endl;
            m_local_goal_pose = m_global_path.poses[len - 1];
            std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;
        }
        else
        {
            if (m_local_goal_pose_index >= len - 1)
            {
                m_local_goal_pose_index = len - 1;
            }
            if (m_local_path.poses.empty())
            {
                for (size_t i = 0; i < mIndexOffsetLocalGoalPose; i++)
                {
                    m_local_path.poses.push_back(m_global_path.poses[i]);
                }
                m_local_goal_pose_initialized = true;
                m_local_goal_pose_index = mIndexOffsetLocalGoalPose;
                m_local_goal_pose = m_global_path.poses[m_local_goal_pose_index];
            }
            int IndexOffset = 0;
            if (m_local_goal_pose_index >= mIndexOffsetLocalGoalPose)
            {
                IndexOffset = mIndexOffsetLocalGoalPose / 2;
            }

            double dist = np::caculate_distance(m_global_path.poses[m_local_goal_pose_index].pose, m_local_start_pose.pose);
            if (dist <= mSetDistLocalStartToGoal)
            {
                ROS_FATAL("dist == %f", dist);
                if (m_local_goal_pose_index >= len - 1)
                {
                    m_local_goal_pose_index = len - 1;
                }
                else if (m_local_goal_pose_index + mIndexOffsetLocalGoalPose >= len - 1) //全局路径最后一段 
                {
                    for (size_t i = m_local_goal_pose_index; i < len; i++)
                    {
                        m_local_path.poses.push_back(m_global_path.poses[i]);
                    }
                    m_local_goal_pose_index = len - 1;
                }
                else
                {
                    for (size_t i = 0; i < mIndexOffsetLocalGoalPose; i++)  //分段赋值给局部路径
                    {
                        m_local_path.poses.push_back(m_global_path.poses[m_local_goal_pose_index + i]);
                    }
                    m_local_goal_pose_index += mIndexOffsetLocalGoalPose;
                }
                m_local_goal_pose_initialized = true;
                m_local_goal_pose = m_global_path.poses[m_local_goal_pose_index];
            }
            m_local_goal_pose.header.frame_id = "map";
            m_local_goal_pose.header.stamp = ros::Time::now();
            m_local_goal_pose.pose.orientation.w = 1;
           
            std::cout << "m_local_goal_pose = \n"<< m_local_goal_pose << std::endl;

            // if (LocalPlan::CalcNoObstcleIndexInPath(m_global_path, m_global_map,mIndexOffsetLocalGoalPose, m_local_goal_pose_index)) //计算非障碍物目标点
            // {
            //     m_local_goal_pose = m_global_path.poses[m_local_goal_pose_index];
            //     return true;
            // }
            // else
            // {
            //     ROS_FATAL("m_local_goal_pose_index = %d, 查找超过最大长度！", m_local_goal_pose_index);
            //     return false;
            // }
            if (LocalPlan::IsObastclePoint(m_local_goal_pose.pose.position, m_global_map))
            {
                std::vector<std::pair<size_t, size_t>> indexObastcleInGlobalPath;                            //计算障碍物在全局路径上的开始和结束索引
                LocalPlan::FindObastacleIndexInPath(m_global_path, m_global_map, indexObastcleInGlobalPath); //TODO local_map
                if (!indexObastcleInGlobalPath.empty())
                {
                    int index = 0;
                    for (size_t i = 0; i < indexObastcleInGlobalPath.size(); i++)
                    {
                        if (m_local_goal_pose_index >= indexObastcleInGlobalPath[i].first && m_local_goal_pose_index <= indexObastcleInGlobalPath[i].second )
                        {
                            index = i;
                        }
                    }

                    geometry_msgs::Point point1 = m_global_path.poses[indexObastcleInGlobalPath[index].first].pose.position;
                    geometry_msgs::Point point2 = m_global_path.poses[indexObastcleInGlobalPath[index].second].pose.position;
                    geometry_msgs::Point pointLeft;
                    geometry_msgs::Point pointRight;
                    double dist = 4;
                    double bound = 60;
                    ros::Rate r(2);
                    for (double d = dist; d <= bound; d += dist)
                    {
                        np::CalcOffsetWith2Point(point1, point2, d, pointLeft, pointRight);
                        bool bLeft = LocalPlan::IsObastclePoint(pointLeft, m_global_map);
                        int colorRGB[3] = {1, 0, 0};
                        geometry_msgs::Pose pose;
                        pose.position = pointLeft;
                        Display(pose, colorRGB);
                        r.sleep();
                        bool bRight = LocalPlan::IsObastclePoint(pointRight, m_global_map);
                        pose.position = pointRight;
                        Display(pose, colorRGB);
                        if (!bLeft || !bRight)
                        {
                            if (!bLeft)
                            {
                                m_local_goal_pose.pose.position = pointLeft;
                            }
                            else
                            {
                                m_local_goal_pose.pose.position = pointRight;
                            }
                            dist = d;
                            break;
                        }
                       
                    }
                    int a = 0;
                    m_local_goal_pose_initialized = true;
                }
            }

           

            return true;

            for (size_t i = 0; i < len; i++)
            {
                if (np::caculate_distance(m_global_path.poses[i].pose, m_local_start_pose.pose) > 90 && np::caculate_distance(m_global_path.poses[i].pose, m_local_start_pose.pose) < 100)
                {
                    m_local_goal_pose.pose = m_global_path.poses[i].pose;
                    m_local_goal_pose.header.frame_id = "map";
                    m_local_goal_pose.header.stamp = ros::Time::now();
                    m_local_goal_pose.pose.orientation.w = 1;
                    std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
                    m_local_goal_pose_initialized = true;
                    return true;
                    break;
                }
            }
        }
    }

    bool global_path_update()
    {
        if (!m_global_map_initialized || !m_global_start_pose_initialized || !m_global_goal_pose_initialized)
        {
            if (!m_global_map_initialized)
            {
                ROS_WARN_THROTTLE(5, "等待global_map： /ow/global_map...");
            }
            if (!m_global_start_pose_initialized)
            {
                ROS_WARN_THROTTLE(5, "等待global_start_pose：/initialpose ...");
            }
            if (!m_global_goal_pose_initialized)
            {
                ROS_WARN_THROTTLE(5, "等待global_goal_pose：move_base_simple/goal ...");
            }
            return false;
        }

        return true;
    }

    void global_start_pose_cb(const geometry_msgs::PoseWithCovarianceStamped &start_pose)
    {
        if (!m_global_map_initialized)
        {
            return;
        }
        m_global_path.poses.clear();
        m_global_path_initialized = false;
        m_global_start_pose.header = start_pose.header;
        m_global_start_pose.pose = start_pose.pose.pose;
        m_global_start_pose_initialized = true;
        mstartPoseTest = m_global_start_pose;
        m_local_goal_pose_index = 0;
        std::cout
            << "m_global_start_pose =\n " << m_global_start_pose << std::endl;
        // ROS_INFO("The m_global_start_pose x = %.2f, y = %.2f", m_global_start_pose.pose.position.x, m_global_start_pose.pose.position.y);
    }

    void local_start_pose_cb(const geometry_msgs::PoseWithCovarianceStamped &start_pose)
    {
        if (!m_local_map_initialized)
        {
            return;
        }
        m_local_path.poses.clear();
        m_local_path_initialized = false;
        m_local_start_pose.header = start_pose.header;
        m_local_start_pose.pose = start_pose.pose.pose;
        m_local_start_pose_initialized = true;
        m_local_goal_pose_index = 0;
        std::cout << "m_local_start_pose = \n"
                  << m_local_start_pose << std::endl;
        // ROS_INFO("The m_local_start_pose x = %.2f, y = %.2f", m_local_start_pose.pose.position.x, m_local_start_pose.pose.position.y);
    }

    void global_goal_pose_cb(const geometry_msgs::PoseStamped &global_goal_pose)
    {
        if (!m_global_map_initialized)
        {
            return;
        }
        m_global_path.poses.clear();
        m_global_path_initialized = false;
        m_global_goal_pose = global_goal_pose;
        m_global_goal_pose_initialized = true;
        m_local_path.poses.clear();
        m_local_path_initialized = false;
        m_local_goal_pose_initialized = false;
        if(mTestLine == 1){
            m_local_goal_pose_index = 0;
        }
        else{
            m_local_goal_pose_index = 100;
        }
       
        ROS_FATAL("m_local_goal_pose_index = %d", m_local_goal_pose_index);
        m_global_goal_pose_vec.push_back(m_global_goal_pose);
        std::cout << "m_global_goal_pose = " << m_global_goal_pose << std::endl;
        // ROS_INFO("The m_global_goal_pose x = %f, y = %f", global_goal_pose.pose.position.x, global_goal_pose.pose.position.y);
    }

    bool LineMotionLocal()
    {
        boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
        if (mPathShae == E_PathShape::Line) //todo sprital line
        {
            m_local_path = m_global_path;
            // m_local_goal_pose_initialized = true;
            return true;
        }
        int len = m_global_path.poses.size();
        std::cout << "The m_global_path len = " << len << std::endl;
        if (len < 1)
        {
            return false;
        }
        if (len < mIndexOffsetLocalGoalPose)
        {
            std::cout << "The  m_global_path.poses[" << len - 1 << "] = " << m_global_path.poses[len - 1] << endl;
            m_local_goal_pose = m_global_path.poses[len - 1];
            for (size_t i = 0; i < len; i++)
            {
                m_local_path.poses.push_back(m_global_path.poses[i]);
            }
            std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;
        }
        else
        {
            
            if (m_local_path.poses.empty())
            {
                for (size_t i = 0; i < mIndexOffsetLocalGoalPose; i++)
                {
                    m_local_path.poses.push_back(m_global_path.poses[i]);
                }
                m_local_goal_pose_index = mIndexOffsetLocalGoalPose;
            }
            int IndexOffset = 0;
            if (m_local_goal_pose_index >= mIndexOffsetLocalGoalPose)
            {
                IndexOffset = mIndexOffsetLocalGoalPose / 2;
            }

            double dist = np::caculate_distance(m_global_path.poses[m_local_goal_pose_index].pose, m_local_start_pose.pose);
            if (dist <= mSetDistLocalStartToGoal)
            {
                m_local_path.poses.clear();
                if (m_local_goal_pose_index + mIndexOffsetLocalGoalPose >= len - 1)
                {
                    for (size_t i = m_local_goal_pose_index - mIndexOffsetLocalGoalPose; i < len; i++)
                    {
                        m_local_path.poses.push_back(m_global_path.poses[i]);
                    }
                    for (size_t i = 0; i < mIndexOffsetLocalGoalPose; i++)
                    {
                        m_local_path.poses.push_back(m_global_path.poses[i]);
                    }
                    m_local_goal_pose_index = mIndexOffsetLocalGoalPose;
                }
                else
                {
                    if (mPathShae == E_PathShape::SpiralLine && m_local_goal_pose_index < mIndexOffsetLocalGoalPose) //todo sprital line
                    {
                        for (size_t i = 0; i < mIndexOffsetLocalGoalPose*10; i++)
                        {
                            m_local_path.poses.push_back(m_global_path.poses[i]);
                        }
                        m_local_goal_pose_index = mIndexOffsetLocalGoalPose*10;
                    }
                    
                    for (size_t i = 0; i < mIndexOffsetLocalGoalPose + 2*IndexOffset; i++)
                    {
                        m_local_path.poses.push_back(m_global_path.poses[m_local_goal_pose_index + i - IndexOffset*2]);
                    }
                    m_local_goal_pose_index += mIndexOffsetLocalGoalPose;
                }
                m_local_goal_pose_initialized = true;
            }
            m_local_goal_pose.pose = m_global_path.poses[m_local_goal_pose_index].pose;
            m_local_goal_pose.header.frame_id = "map";
            m_local_goal_pose.header.stamp = ros::Time::now();
            m_local_goal_pose.pose.orientation.w = 1;
            std::cout << "m_local_goal_pose = \n"
                      << m_local_goal_pose << std::endl;
           
        }
        return true;
    }

#pragma endregion

#pragma region //local_plan_thread

    void local_plan_thread()
    {
        ROS_INFO("The Decision::local_plan_thread 被执行 ..."); //local_map
        m_local_map_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>("/ow/local_map", 1, &Decision::local_map_cb, this);
        m_local_start_pose_sub = m_nh.subscribe("/initialpose", 1, &Decision::local_start_pose_cb, this); //TODO 只留一个
        // m_local_goal_pose_sub = m_nh.subscribe("move_base_simple/goal", 1, &Decision::local_goal_pose_cb, this);
        
        m_local_raw_path_pub = m_nh.advertise<nav_msgs::Path>("local_raw_path", 1);
        m_local_path_pub = m_nh.advertise<nav_msgs::Path>("/ow/local_path", 1);
        m_system_command_pub = m_nh.advertise<pkg_nav::system_command>("/ow/system_command", 10);  //
        mOdometryPub = m_nh.advertise<nav_msgs::Odometry>("/ow/odometry", 1);
        ROS_WARN("The local_plan_thread 已经启动，请设置局部地图及起始点....");

        pkg_nav::local_pathGoal goal; //定义要做的目标
        pkg_nav::global_path::Request request;
        pkg_nav::global_path::Response response;
        unsigned count = 1;
        // 写文件
        // 基于当前系统的当前日期/时间
        time_t now = time(0); //now = 1638537838
        // 把 now 转换为字符串形式
        char *dt = ctime(&now); //Fri Dec  3 21:34:50 2021
        dt[strlen(dt) - 1] = 0;
        string fileName = dt;
        np::erase_all_space(fileName);
        fileName = "local_path_" + fileName;
        bool isUpdateLocalPath = false;

        ros::Rate loop_rate(1);
        while (ros::ok)
        {
            ros::spinOnce();
            // if (!m_local_path.poses.empty())
            // {
            //     mIndex += 40;
            //     if (mIndex >= m_local_path.poses.size() - 1)
            //     {
            //         mIndex = m_local_path.poses.size() - 1;
            //         mIndex = 0;
            //     }
            //     //======mOdometry msg===
            //     mOdometry.header.frame_id = "map";
            //     mOdometry.pose.pose = m_local_path.poses[mIndex].pose;
            //     odometry_cb(mOdometry);
            //     // mOdometryPub.publish(mOdometry);
            //     int colorRGB[3] = {0, 1, 0};
            //     Display(mOdometry.pose.pose, colorRGB);
            // }
            
            // int run_state = m_run_state;
            m_system_command.runCommand = m_run_state;
            m_system_command_pub.publish(m_system_command);
            
            if (m_run_state != E_SystemRunState::IN_planning)
            {
                loop_rate.sleep();
                continue;
            }
            mActDistLocalStartToGoal = np::caculate_distance(m_local_start_pose.pose, m_global_goal_pose.pose);
            if (local_path_update())
            {
                if (mTestLine == 1)
                {
                     if(Decision::LineMotionLocal())
                     {
                         if (m_local_goal_pose_initialized && isUpdateLocalPath)
                         {
                             mIndex = 0;
                         }
                         m_local_goal_pose_initialized = true;
                     }
                }
                else
                {
                    if (mActDistLocalStartToGoal > mSetDistLocalStartToGoal / 2)
                    {
                        if (Decision::get_local_path())
                        {
                            if (m_local_goal_pose_initialized && isUpdateLocalPath)
                            {
                                mIndex = 0;
                            }
                        }
                    }
                }
               
                if (!m_local_goal_pose_initialized)
                {
                    ROS_WARN("局部目标位姿m_local_goal_pose_initialized 还未更新....");
                    // m_local_path_initialized = false;
                    loop_rate.sleep();
                    continue;
                }

                if (mTestLine == 1 )
                {
                    m_local_path_initialized = m_local_goal_pose_initialized;
                    m_local_goal_pose_initialized = false;
                }
                else
                {
                    request.num = 10;
                    request.start_pose = m_local_start_pose;
                    request.goal_pose = m_local_goal_pose;
                    if (mActDistLocalStartToGoal > mSetDistLocalStartToGoal/4)
                    {
                        for (auto pose : m_local_path.poses)
                        {
                            if (LocalPlan::IsObastclePoint(pose.pose.position, m_global_map))
                            {
                                m_local_goal_pose_initialized = true;
                                isUpdateLocalPath = true;
                                break;
                            }
                            else
                            {
                                m_local_path_initialized = true;
                            }
                        }
                        if (m_local_goal_pose_initialized && isUpdateLocalPath)
                        {
                            m_local_path_initialized = Decision::local_plan_response(request, response);
                            m_local_path = response.global_path;
                            m_local_goal_pose_initialized = false;
                            isUpdateLocalPath = false;
                        }
                        // for (auto pose : m_local_path.poses)
                        // {
                        //     if (LocalPlan::IsObastclePoint(pose.pose.position, m_global_map))
                        //     {
                        //         m_local_goal_pose_initialized = true;
                        //         isUpdateLocalPath = true;
                        //         break;
                        //     }
                        //     else
                        //     {
                        //         m_local_path_initialized = true;
                        //     }
                        // }
                    }
                }

                if (m_local_path_initialized)
                {
                    
                    // if (mTestLine != 1)
                    // {
                    //     m_local_path = response.global_path;
                    // }
                    m_local_path.header.frame_id = "map";         
                    m_local_path.header.seq = count;              // TODO
                    m_local_path.header.stamp = ros::Time::now(); // TODO
                    m_local_raw_path.header.frame_id = "map";
                    m_local_raw_path.header.stamp = ros::Time::now();
                    m_local_raw_path_pub.publish(m_local_raw_path);
                    m_local_path_pub.publish(m_local_path); //ow/local_path
                    m_local_path_initialized = false;

                    WriteFile(fileName, m_local_path, count);
                    ROS_WARN("The m_local_path_pub count = %d", count++);

                    // goal.num = 10;
                    // goal.runCommand = m_run_state;
                    // goal.local_path = m_local_path;

                    // m_local_path_client.waitForServer(); //等待服务器初始化完成  //程序堵塞直到server启动
                    // //发送目标至服务器
                    // m_local_path_client.sendGoal(goal,
                    //                              boost::bind(&Decision::done_cb, this, _1, _2),
                    //                              boost::bind(&Decision::active_cb, this),
                    //                              boost::bind(&Decision::feedback_cb, this, _1));
                    // m_local_path_client.waitForResult(ros::Duration(5)); //等待结果完成，可以设置等待时间
                    // m_local_start_pose_initialized = false;
                }
                else
                {
                    if (mActDistLocalStartToGoal <= mSetDistLocalStartToGoal / 4)
                    {
                        if (m_err_state != E_SystemErrorState::Err_local_planning && m_err_state != E_SystemErrorState::Err_local_planning)
                        {
                            continue;
                        }
                    }
                    ROS_FATAL("The local_plan_response 执行失败，正在尝试重新规划");
                    m_global_path_initialized = false;
                    m_global_start_pose_initialized = false;
                    m_global_path.poses.clear();
                    m_local_path.poses.clear();
                    m_local_path.poses.push_back(m_local_start_pose);
                    m_local_path_pub.publish(m_local_path);
                    m_global_path_pub.publish(m_global_path);
                    m_err_state = E_SystemErrorState::Err_local_planning;
                    // m_run_state = E_SystemRunState::IN_emergency_stop;
                    return;
                }
            }
            loop_rate.sleep();
        }
        // m_local_path_client.waitForResult(ros::Duration(5)); ///等待结果，时间间隔5秒   会导致堵塞
        // //根据返回结果，做相应的处理
        // if (m_local_path_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        //     printf("Yay! The local path server has finished !");
        // else
        // {
        //     ROS_INFO("Cancel Goal!");
        //     m_local_path_client.cancelAllGoals();
        // }
        // printf("Current State: %s\n", m_local_path_client.getState().toString().c_str());
        // m_local_path_pub.publish(m_local_path); //ow/local_path
    }

    // bool 返回值用于标志是否处理成功
    bool local_plan_response(pkg_nav::global_path::Request &request, pkg_nav::global_path::Response &response)
    {
        bool is_point_robot = true; // 是否为点状机器人
        // 可视化非常耗费时间, 若不需要查看A*算法中的探索过程,
        // 建议将可视化的标志位置为false
        bool is_visualization = false;  // 是否需要可视化中间过程
        bool is_eight_connected = true; // 每个节点的方向是否为八连通
        ros::NodeHandle nh;
        std::unique_ptr<AStar> a_star_ptr = std::make_unique<AStar>(nh, is_visualization, is_eight_connected);
        AStarResult a_star_result;

        nav_msgs::Path path;
        // a_star_ptr->SetMap(m_local_map); //TODO
        a_star_ptr->SetMap(m_global_map); //TODO
        if (!is_point_robot)
        {
            // 如果不是点机器人, 则需要设置长宽等参数
            a_star_ptr->SetVhicleParams(5.0, 5.0, 3.0);
        }
        ros::Time start_time = ros::Time::now();
        if (a_star_ptr->SearchPath(request.start_pose.pose.position.x, request.start_pose.pose.position.y, request.goal_pose.pose.position.x, request.goal_pose.pose.position.y, &a_star_result))
        {
            float delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[查找local path delta_time =  " << delta_time << " s]!" << std::endl;
            size_t len = a_star_result.x.size();
           
            std::vector<std::pair<double, double>> raw_point2d;
            std::vector<std::pair<double, double>> upperBounds;
            std::vector<std::pair<double, double>> lowerBounds;
            std::vector<double> bounds;
            geometry_msgs::PoseStamped raw_pose;
            nav_msgs::Path raw_path;
            double x, y;
            int distBound = 10;
            double boundLU = 2;
            int distPerPixel = 1 / m_global_map.info.resolution;
            for (int i = 0; i < len; i++)
            {
                bool bXLeft = true;
                bool bXRight = true;
                bool bYDown = true;
                bool bYUp = true;
                bool bXYLeftDown = true;
                bool bXYLeftUp = true;
                bool bXYRightDown = true;
                bool bXYRightUp = true;
                int xBound = 0;
                int yBound = 0;
                int xyBound = 0;
                int index_x = a_star_result.x[i] / m_global_map.info.resolution;
                int index_y = a_star_result.y[i] / m_global_map.info.resolution;
                const int index = CalcIndex(index_x, index_y, m_global_map); // 计算在一维数组中的索引

                //========== x bound =======================
                for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x left
                {
                    if (m_global_map.data[index - j] != 0)
                    {
                        bXLeft = false;
                        break;
                    }
                }
                for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x right
                {
                    if (m_global_map.data[index + j] != 0)
                    {
                        bXRight = false;
                        break;
                    }
                }
                if (bXLeft && !bXRight)
                {
                    xBound = -distBound / 3;
                }
                else if (!bXLeft && bXRight)
                {
                    xBound = distBound / 3;
                }
                else
                {
                    xBound = 0;
                }

                //========== y bound =======================
                if (xBound != 0)
                {
                    for (size_t j = 1; j <= distBound / 2 * distPerPixel; j++) //y down
                    {
                        if (m_global_map.data[index + xBound / m_global_map.info.resolution - j * m_global_map.info.width] != 0)
                        {
                            bYDown = false;
                            break;
                        }
                    }
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //y up
                    {
                        if (m_global_map.data[index + xBound / m_global_map.info.resolution + j * m_global_map.info.width] != 0)
                        {
                            bYUp = false;
                            break;
                        }
                    }
                    if (bYDown && !bYUp)
                    {
                        yBound = -distBound / 3;
                    }
                    else if (!bYDown && bYUp)
                    {
                        yBound = distBound / 3;
                    }
                    else
                    {
                        yBound = 0;
                    }
                }
                else
                {
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //y down
                    {
                        if (m_global_map.data[index - j * m_global_map.info.width] != 0)
                        {
                            bYDown = false;
                            break;
                        }
                    }
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //y up
                    {
                        if (m_global_map.data[index + j * m_global_map.info.width] != 0)
                        {
                            bYUp = false;
                            break;
                        }
                    }
                    if (bYDown && !bYUp)
                    {
                        yBound = -distBound / 3;
                    }
                    else if (!bYDown && bYUp)
                    {
                        yBound = distBound / 3;
                    }
                    else
                    {
                        yBound = 0;
                    }
                }

                //========== -x -y <--> +x +y bound =======================
                if (xBound == 0 && yBound == 0)
                {
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x left
                    {
                        if (m_global_map.data[index - j - j * m_global_map.info.width] != 0)
                        {
                            bXYLeftDown = false;
                            break;
                        }
                    }
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x right
                    {
                        if (m_global_map.data[index + j + j * m_global_map.info.width] != 0)
                        {
                            bXYRightUp = false;
                            break;
                        }
                    }
                    if (bXYLeftDown && !bXYRightUp)
                    {
                        xBound = -distBound / 3;
                        yBound = -distBound / 3;
                    }
                    if (bXYRightUp && !bXYLeftDown)
                    {
                        xBound = distBound / 3;
                        yBound = distBound / 3;
                        // ROS_FATAL("!bXYLeftDown && bXYRightUp");
                    }

                    //========== -x +y <--> +x -y bound =======================
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x left
                    {
                        if (m_global_map.data[index - j + j * m_global_map.info.width] != 0)
                        {
                            bXYLeftUp = false;
                            break;
                        }
                    }
                    for (size_t j = 1; j <= distBound/2 * distPerPixel; j++) //x right
                    {
                        if (m_global_map.data[index + j - j * m_global_map.info.width] != 0)
                        {
                            bXYRightDown = false;
                            break;
                        }
                    }
                    if (bXYLeftUp && !bXYRightDown)
                    {
                        xBound = -distBound / 3;
                        yBound = distBound / 3;
                    }
                    else if (!bXYLeftUp && bXYRightDown)
                    {
                        xBound = distBound / 3;
                        yBound = -distBound / 3;
                    }
                }
                x = a_star_result.x[i] + xBound * 1;
                y = a_star_result.y[i] + yBound * 1;
                raw_pose.pose.position.x = a_star_result.x[i];
                raw_pose.pose.position.y = a_star_result.y[i];
                raw_path.poses.push_back(raw_pose);
                raw_point2d.push_back(std::make_pair(x, y));
                upperBounds.push_back(std::make_pair(boundLU, boundLU));
                lowerBounds.push_back(std::make_pair(boundLU, boundLU));
            }
            upperBounds[0].first = 0.1;
            upperBounds[0].second = 0.1;
            upperBounds[upperBounds.size() - 1].first = 0;
            upperBounds[upperBounds.size() - 1].second = 0;
            lowerBounds[0].first = 0.1;
            lowerBounds[0].second = 0.1;
            lowerBounds[lowerBounds.size() - 1].first = 0;
            lowerBounds[lowerBounds.size() - 1].second = 0;
            m_global_raw_path = raw_path;
            FemPosDeviationSqpOsqpInterface solver;
            std::cout << "OSQP RUNNING..." << std::endl;
            solver.set_ref_points(raw_point2d);
            solver.set_bounds_around_refs(upperBounds, lowerBounds);
            if (!solver.Solve())
            {
                std::cout << "failture" << std::endl;
                ROS_FATAL("The FemPosDeviationSqpOsqpInterface solver is  failture !");
                response.global_path.poses.clear();
                return false;
            }
            std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();
            geometry_msgs::PoseStamped pose;
            std::cout << "OSQP END!!!" << std::endl;
            for (size_t i = 0; i < opt_xy.size(); i++)
            {
                pose.pose.position.x = opt_xy[i].first;
                pose.pose.position.y = opt_xy[i].second;
                path.poses.push_back(std::move(pose));
            }
            response.global_path = path;
            ROS_WARN("The test for local_plan_response of response.global_path is normal");
            delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[局部路径总时间 local path delta_time =  " << delta_time << " s]!" << std::endl;
            return true;
        }
        else
        {
            ROS_FATAL("局部路径查找失败 ！");
            return false;
        }
       
    }

    bool local_path_update()
    {
        if (!m_local_map_initialized || !m_local_start_pose_initialized || !m_global_path_initialized)
        {
            if (!m_local_map_initialized)
            {
                ROS_WARN("等待local_map： /ow/local_map...");
            }
            if (!m_local_start_pose_initialized)
            {
                ROS_WARN("等待local_start_pose：/initialpose ...");
            }
            if (!m_global_path_initialized)
            {
                ROS_WARN("等待 m_global_path_initialized ..."); //TODO
            }
            return false;
        }
        return true;
    }

    void Display(geometry_msgs::Pose pose, int colorRGB[])
    {
        //实例化一个Marker
        // visualization_msgs::Marker mMarker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        // 设置frame ID 和 时间戳
        mMarker.header.frame_id = "map";
        mMarker.header.stamp = ros::Time::now();
        // Set the namespace and id for this mMarker.  This serves to create a unique ID
        // Any mMarker sent with the same namespace and id will overwrite the old one
        // 为这个mMarker设置一个独一无二的ID，一个mMarker接收到相同ns和id就会用新的信息代替旧的
        mMarker.ns = "basic_shapes";
        mMarker.id = 0;
        // Set the mmMarker type.  Initially this is sphere, and cycles between that and SPHERE, ARROW, and CYLINDER
        mMarker.type = mShape;
        // Set the mmMarker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        mMarker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the mMarker.  This is a full 6DOF pose relative to the frame/time specified in the header
        // 设置mMarker的位置
        mMarker.pose.position.x = pose.position.x;
        mMarker.pose.position.y = pose.position.y;
        mMarker.pose.position.z = 0;
        mMarker.pose.orientation.x = 0.0;
        mMarker.pose.orientation.y = 0.0;
        mMarker.pose.orientation.z = 0.0;
        mMarker.pose.orientation.w = 1.0;
        // Set the scale of the mMarker -- 1x1x1 here means 1m on a side
        // 设置mMarker的大小
        mMarker.scale.x = 5;
        mMarker.scale.y = 5;
        mMarker.scale.z = 5;
        // Set the color -- be sure to set alpha to something non-zero!
        // 设置mMarker的颜色
        mMarker.color.r = colorRGB[0]; // 红色（R）0 到 255 间的整数，代表颜色中的红色成分
        mMarker.color.g = colorRGB[1]; //绿色（G）0 到 255 间的整数，代表颜色中的绿色成分。
        mMarker.color.b = colorRGB[2]; //蓝色（B）0 到 255 间的整数，代表颜色中的蓝色成分。
        mMarker.color.a = 1.0;         // 透明度（A）取值 0~1 之间， 代表透明度。
        //取消自动删除
        mMarker.lifetime = ros::Duration();
        // Publish the mMarker
        // 必须有订阅者才会发布消息
        // while (mMarkerPub.getNumSubscribers() < 1)
        // {
        //     if (!ros::ok())
        //     {
        //         return;
        //     }
        //     ROS_WARN_ONCE("Please create a subscriber to the mMarker");
        //     sleep(1);
        // }
        mMarkerPub.publish(mMarker);
    }
#pragma endregion
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    //执行 ros 节点初始化
    ros::init(argc, argv, "ow_decision_node");
    ROS_INFO("The program of decision_node is running ...");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    Decision decision_obj("/ow/local_path", true);
    // AstarNavi node;
    // node.run();
    ros::spin();
}

int main1(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    //执行 ros 节点初始化
    ros::init(argc, argv, "ow_decision_node");
    ROS_INFO("The program of decision_node is running ...");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    // Decision decision_obj("/ow/local_path", true);
    // AstarNavi node;
    // node.run();
    // 写文件
    // ofstream outFile;
    // outFile.open("/home/cyb/lq/global_path_data.csv", ios::out); // 打开模式可省略
    // outFile << "name" << ',' << "age" << ',' << "hobby" << endl;
    // outFile << "Mike" << ',' << 18 << ',' << "paiting" << endl;
    // outFile << "Tom" << ',' << 25 << ',' << "football" << endl;
    // outFile << "Jack" << ',' << 21 << ',' << "music" << endl;
    // ROS_WARN("The outFile is running ...");
    // outFile.close();

    // // 读文件
    // ifstream inFile("data.csv", ios::in);
    // string lineStr;
    // vector<vector<string>> strArray;
    // while (getline(inFile, lineStr))
    // {
    //     // 打印整行字符串
    //     cout << lineStr << endl;
    //     ROS_WARN("The outFile is running ...");
    //     // 存成二维表结构
    //     stringstream ss(lineStr);
    //     string str;
    //     vector<string> lineArray;
    //     // 按照逗号分隔
    //     while (getline(ss, str, ','))
    //         lineArray.push_back(str);
    //     strArray.push_back(lineArray);
    // }
    // char *buffer;
    // //也可以将buffer作为输出参数
    // if ((buffer = getcwd(NULL, 0)) == NULL)
    // {
    //     perror("getcwd error");
    //     ROS_FATAL("perror");
    // }
    // else
    // {
    //     printf("%s\n", buffer);
    //     ROS_FATAL("buffer = %s", buffer);
    // }
    // ofstream outFile;
    // string cwdStr = buffer;
    // string pathStr = "global_path";
    // ostringstream cwdss;
    // std::cout << cwdStr.c_str() << std::endl;
    // cwdss << cwdStr << "/" << pathStr << "_data.csv";
    // ROS_FATAL("cwdStr.c_str()");
    // std::cout << cwdss.str() << std::endl;
    // outFile.open(cwdss.str(), ios::app); // 打开模式可省略
    // // outFile.open(xss.str(), ios::out); // 打开模式可省略
    // string s = "陈";
    // int a = 10;
    // double b = 1234;
    // ostringstream oss;
    // oss << s << "," << a << "," << b << ",";
    // outFile << oss.str() << endl;
    // cout << oss.str() << endl;
    // outFile.close();
    // free(buffer);

    // time_t timer = time(NULL);
    // struct tm *localtm = localtime(&timer);
    // std::cout << localtm->tm_year << std::endl;
    // std::cout << localtm->tm_mon << std::endl;
    // std::cout << localtm->tm_mday << std::endl;
    // std::cout << localtm->tm_hour << std::endl;
    // std::cout << localtm->tm_min << std::endl;
    // ostringstream cwdss;
    // cwdss << localtm->tm_year << "_" << localtm->tm_mon << localtm->tm_mday;
    // std::cout << cwdss.str() << std::endl;

    // 基于当前系统的当前日期/时间
    // time_t now = time(0); //now = 1638537838
    // // 把 now 转换为字符串形式
    // char *dt = ctime(&now); //Fri Dec  3 21:34:50 2021
    // dt[strlen(dt) - 1] = 0;
    // string str = dt;
    // np::erase_all_space(str);
    // cwdss << "act_path_" << str << ".csv";
    // ofstream outFile;
    // outFile.open(cwdss.str(), ios::out); // 打开模式可省略
    // outFile << cwdss.str() << endl;
    // cout << cwdss.str() << endl;
    // outFile.close();

    // std::cout << "cwdss.str() = " << cwdss.str() << std::endl;
    // cout << "本地日期和时间：" << dt << endl;

    // 把 now 转换为 tm 结构
    // tm *gmtm = gmtime(&now);
    // dt = asctime(gmtm);
    // cout << "UTC 日期和时间：" << dt << endl;

    // ros::spin();
    // std::vector<geometry_msgs::Point> pointV1;  //Raw
    // std::vector<geometry_msgs::Point> pointInV1; //Inner
    // std::vector<geometry_msgs::Point> pointOutV1; //outer
    // geometry_msgs::Point point;
    // geometry_msgs::Point PointIn;
    // geometry_msgs::Point PointOut;
    // point.x = 0;
    // point.y = 0;
    // double xArr[21] = {0, 1, 2, 3, 4, 4, 4, 3, 2, 0, -2, -3, -4, -4, -4, -3, -2, -1, 0, 1, 2};
    // double yArr[21] = {0, 1, 2, 2, 2, 3, 4, 5, 6, 6,  6,  5,  4,  3,  2,  2,  2,  1, 0, 1, 2};
    // for (size_t i = 0; i < 21; i++)
    // {
    //     point.x = xArr[i];
    //     point.y = yArr[i];
    //     pointV1.push_back(point);
    // }
    // double dist = 1;
    // for (size_t i = 0; i < pointV1.size() - 3; i++)
    // {
    //     np::CalcOffsetPoint(pointV1[i], pointV1[i + 1], pointV1[i + 2], dist,PointIn, PointOut);
    //     pointInV1.push_back(PointIn);
    //     pointOutV1.push_back(PointOut);
    // }
    // std::cout << "pointInV1 = " << std::endl;
    // for (auto var : pointInV1)
    // {
    //     std::cout << var.x << " " << var.y << " " << std::endl;
    // }
    // std::cout << "pointOutV1 = " << std::endl;
    // for (auto var : pointOutV1)
    // {
    //     std::cout << var.x << " " << var.y << " " << std::endl;
    // }

        return 0;
}
