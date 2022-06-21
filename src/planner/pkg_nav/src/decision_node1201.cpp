
#include<pkg_nav/decision_node.h>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <cmath>
#include "actionlib/client/simple_action_client.h"
#include "actionlib/server/simple_action_server.h"
#include "pkg_nav/local_pathAction.h"
// #include "pkg_nav/global_pathAction.h"
#include "pkg_nav/global_path.h"
#include "pkg_nav/astar_search.h"
#include <boost/thread.hpp>
#include <eigen3/Eigen/Dense>

#include <tf/transform_listener.h>
#include <autoware_msgs/LaneArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

#include <visualization_msgs/MarkerArray.h>
#include "pkg_nav/a_star_search.h"
#include "pkg_nav/osqp_problem.h"
#include "pkg_nav/smoothosqpproblem.h"
#include "pkg_nav/system_command.h"

using namespace std;
using namespace Eigen;
// global_pathAction;
typedef actionlib::SimpleActionClient<pkg_nav::local_pathAction> Client;

class Decision
{
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
    int m_local_goal_pose_index = 0;

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

public:
    float x = 100;
    float y = 100;

public:
    Decision(const std::string client_name, bool flag = true) : m_local_path_client(client_name, flag)
    {
        ROS_INFO("The Decision::Decision() fun is executed!");
        m_run_state = E_SystemRunState::IN_idle;
        ros::Rate cycle_rate(1);
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
            m_Private_nh.param("distCheckLocalStartToGoal", mSetDistLocalStartToGoal, 15.0); //10m
            m_Private_nh.param("indexOffsetLocalGoalPose", mIndexOffsetLocalGoalPose, 100);
            m_local_goal_pose_index = 0;
        }
        else
        {
            mPathShae = E_PathShape::AStar;
            m_local_goal_pose_index = 100;
        }
        m_global_plan_thread = new boost::thread(boost::bind(&Decision::global_plan_thread, this));
        m_local_plan_thread = new boost::thread(boost::bind(&Decision::local_plan_thread, this));
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
    }

    bool lineMotionGlobal()
    {
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
            if (!m_global_path_initialized)
            {
                mstartPoseTest = m_global_start_pose;
            }
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
            if (!m_global_path_initialized)
            {
                mstartPoseTest = m_global_start_pose;
            }
            float dx = m_global_goal_pose.pose.position.x - mstartPoseTest.pose.position.x;
            float dy = m_global_goal_pose.pose.position.y - mstartPoseTest.pose.position.y;
            int countPath = sqrt(dx * dx + dy * dy);
            geometry_msgs::PoseStamped tempPose;
            for (size_t j = 0; j < countPath; j++)
            {
                tempPose.pose.position.x = mstartPoseTest.pose.position.x + j * dx / countPath;
                tempPose.pose.position.y = m_global_start_pose.pose.position.y + j * dy / countPath;
                path.poses.push_back(tempPose);
            }
            path.poses.push_back(m_global_goal_pose_vec[indexGoalPos]);

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
        ros::Rate loop_rate(1);
        while (ros::ok)
        {
            if (m_run_state != E_SystemRunState::IN_planning)
            {
                loop_rate.sleep();
                continue;
            }
            ros::spinOnce();
            if (count == 1)
            {
                ;
                // Decision::get_start_pose();
                // Decision::get_goal_pose();
            }
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
                    m_global_path.header.frame_id = "map"; //TODO
                    m_global_path.header.seq = count;
                    m_global_path.header.stamp = ros::Time::now();
                    m_global_raw_path.header.frame_id = "map";
                    // for (size_t i = 0; i < 10; i++)
                    // {
                    //     std::cout << i + 1 << " = " << m_global_raw_path.poses[i].pose.position << std::endl;
                    // }

                    m_global_raw_path_pub.publish(m_global_raw_path);
                    m_global_path_pub.publish(m_global_path);
                    ROS_WARN("The m_global_path_pub count = %d", count++);
                    ROS_INFO("The m_global_client 已经获取到 global_path ：sum = %d", global_path_request.response.sum);
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
        if (a_star_ptr->SearchPath(start_pose.position.x, start_pose.position.y, goal_pose.position.x, goal_pose.position.y, &a_star_result))
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
                std::cout << "OSQP RUNNING..." << std::endl;
                std::vector<std::pair<double, double>> raw_point2d;
                std::vector<std::pair<double, double>> upperBounds;
                std::vector<std::pair<double, double>> lowerBounds;
                std::vector<double> bounds;
                geometry_msgs::PoseStamped raw_pose;
                nav_msgs::Path raw_path;
                double x, y;
                int distBound = 20 ;
                int distPerPixel = 1 / m_global_map.info.resolution;

                for (int i = 0; i < len; i += 4)
                {
                    x = a_star_result.x[i];
                    y = a_star_result.y[i];
                    bool bXUpper = true;
                    bool bXLower = true;
                    bool bYUpper = true;
                    bool bYLower = true;
                    double xBound[2] = {0, 0};
                    double yBound[2] = {0, 0};
                    int k = 0;
                    int index_x = x / m_global_map.info.resolution;
                    int index_y = y / m_global_map.info.resolution;
                    const int index = CalcIndex(index_x, index_y, m_global_map);           // 计算在一维数组中的索引
                    //==========bXUpper=======================
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //x upper right
                    {
                        if (m_global_map.data[index + j] != 0)
                        {
                            bXUpper = false;
                            break;
                        }
                    }
                    if (bXUpper)
                    {
                        xBound[0] = distBound/2;
                        // ROS_FATAL("xBound[0] = %f ", xBound[0]);
                    }
                    else
                    {
                        bXUpper = true;
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x upper right
                        {
                            if (m_global_map.data[index - j] != 0)
                            {
                                bXUpper = false;
                                break;
                            }
                        }
                        if (bXUpper)
                        {
                            xBound[0] = -distBound / 2;
                            // ROS_FATAL("-xBound[0] = %f ", xBound[0]);
                        }
                        else{
                            xBound[0] = 0;
                        }
                    }

                    //==========bXLower=======================
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //x lower left
                    {
                        if (m_global_map.data[index - j] != 0)
                        {
                            bXLower = false;
                            break;
                        }
                    }
                    if (bXLower)
                    {
                        xBound[1] = distBound / 2 + 1;
                        // ROS_FATAL("-xBound[1] = %f ", xBound[1]);
                    }
                    else
                    {
                        bXLower = true;
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //x lower right
                        {
                            if (m_global_map.data[index + j] != 0)
                            {
                                bXLower = false;
                                break;
                            }
                        }
                        if (bXLower)
                        {
                            xBound[1] = -distBound / 2 + 1;
                            // ROS_FATAL("xBound[1] = %f ", xBound[1]);
                        }
                        else
                        {
                            xBound[1] = 0;
                        }
                    }

                    //==========bYUpper=======================
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //y up
                    {
                        if (m_global_map.data[index + j*m_global_map.info.width] != 0)
                        {
                            bYUpper = false;
                            break;
                        }
                    }
                    if (bYUpper)
                    {
                        yBound[0] = distBound / 2;
                      
                    }
                    else
                    {
                        bYUpper = true;
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y down
                        {
                            if (m_global_map.data[index - j * m_global_map.info.width] != 0)
                            {
                                bYUpper = false;
                                break;
                            }
                        }
                        if (bYUpper)
                        {
                            yBound[0] = -distBound / 2;
                           
                        }
                        else
                        {
                            yBound[0] = 0;
                        }
                    }

                    //==========bYLower=======================
                    for (size_t j = 1; j <= distBound * distPerPixel; j++) //y down
                    {
                        if (m_global_map.data[index - j * m_global_map.info.width] != 0)
                        {
                            bYLower = false;
                            break;
                        }
                    }
                    if (bYLower)
                    {
                        yBound[1] = distBound / 2 + 1;
                        
                    }
                    else
                    {
                        bYLower = true;
                        for (size_t j = 1; j <= distBound * distPerPixel; j++) //y up
                        {
                            if (m_global_map.data[index + j * m_global_map.info.width] != 0)
                            {
                                bYLower = false;
                                break;
                            }
                        }
                        if (bYLower)
                        {
                            yBound[1] = -distBound / 2 + 1;
                        }
                        else
                        {
                            yBound[1] = 0;
                        }
                    }

                    raw_pose.pose.position.x = a_star_result.x[i];
                    raw_pose.pose.position.y = a_star_result.y[i];
                    raw_path.poses.push_back(raw_pose);
                    raw_point2d.push_back(std::make_pair(a_star_result.x[i], a_star_result.y[i]));
                    upperBounds.push_back(std::make_pair(xBound[0], yBound[0]));
                    lowerBounds.push_back(std::make_pair(xBound[1], yBound[1]));
                }
                // upperBounds[0].first = 0.1;
                // upperBounds[0].second = 0.1;
                // lowerBounds[len - 1].first = 0.1;
                // lowerBounds[len - 1].second = 0.1;
                m_global_raw_path = raw_path;
                FemPosDeviationSqpOsqpInterface solver;
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

    bool get_start_pose()  //测试用
    {
        geometry_msgs::PoseWithCovarianceStamped start_pose;
        start_pose.header.frame_id = "map";
        start_pose.pose.pose.position.x = x;
        start_pose.pose.pose.position.y = y;
        if (x < m_global_goal_pose.pose.position.x || y < m_global_goal_pose.pose.position.y)
        {
            if (x < m_global_goal_pose.pose.position.x)
                x += 10;
            if (y < m_global_goal_pose.pose.position.y)
                y += 10;
        }
        else
        {
            x = 100;
            y = 100;
        }

        // m_global_start_pose = start_pose
        // m_global_start_pose_pub.publish(start_pose);

        return true;
    }

    bool get_goal_pose()
    {
        geometry_msgs::PoseStamped goal_pose;
        goal_pose.pose.position.x = 500;
        goal_pose.pose.position.y = 500;
        // m_global_goal_pose = goal_pose;
        m_global_goal_pose_pub.publish(goal_pose);
        return true;
    }

    bool get_local_goal_pose()
    {
        boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
        int len = m_global_path.poses.size();
        std::cout << "The m_global_path len = " << len << std::endl;
        if (len < 1)
        {
            return false;
        }
        // if (Decision::caculate_distance(m_global_path.poses[len - 1].pose, m_local_start_pose.pose) < 100)
        if (len < mIndexOffsetLocalGoalPose)
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
            m_local_goal_pose.pose = m_global_path.poses[m_local_goal_pose_index].pose;
            double dist = Decision::caculate_distance(m_global_path.poses[m_local_goal_pose_index].pose, m_local_start_pose.pose);
            if (dist <= 50)
            {
                m_local_goal_pose_index += 50;
                if (m_local_goal_pose_index >= len -1)
                {
                    m_local_goal_pose_index = len - 1;
                }
                m_local_goal_pose.pose = m_global_path.poses[m_local_goal_pose_index].pose;
            }
            m_local_goal_pose.header.frame_id = "map";
            m_local_goal_pose.header.stamp = ros::Time::now();
            m_local_goal_pose.pose.orientation.w = 1;
            std::cout << "m_local_goal_pose = \n"
                      << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;

            for (size_t i = 0; i < len; i++)
            {
                if (Decision::caculate_distance(m_global_path.poses[i].pose, m_local_start_pose.pose) > 90 && Decision::caculate_distance(m_global_path.poses[i].pose, m_local_start_pose.pose) < 100)
                {
                    // std::cout << "m_global_path.poses[" << i + 1 << "]" << m_global_path.poses[i] << endl;
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
        m_local_goal_pose_index = 0;
        ROS_FATAL("m_local_goal_pose_index = %f", m_local_goal_pose_index);
        m_global_goal_pose_vec.push_back(m_global_goal_pose);
        std::cout << "m_global_goal_pose = " << m_global_goal_pose << std::endl;
        // ROS_INFO("The m_global_goal_pose x = %f, y = %f", global_goal_pose.pose.position.x, global_goal_pose.pose.position.y);
    }

    void local_goal_pose_cb(const geometry_msgs::PoseStamped &local_goal_pose)
    {
        if (!m_local_map_initialized)
        {
            return;
        }
        m_local_path.poses.clear();
        m_local_path_initialized = false;
        m_local_goal_pose = local_goal_pose;
        // m_local_goal_pose_initialized = true;
        std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
        // ROS_INFO("The m_local_goal_pose x = %f, y = %f", m_local_goal_pose.pose.position.x, m_local_goal_pose.pose.position.y);
    }

    bool LineMotionLocal()
    {
        if (mPathShae == E_PathShape::AStar)
        {
            boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
            m_local_path = m_global_path;
            m_local_goal_pose_initialized = true;
            return true;
        }

        boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
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
            if (mPathShae == E_PathShape::AStar)
            {
                // m_local_path = m_global_path;
                m_local_path.poses.clear();
                m_local_goal_pose_index = mIndexOffsetLocalGoalPose;
                for (size_t i = 0; i < m_local_goal_pose_index; i++)
                {
                    m_local_path.poses.push_back(m_global_path.poses[i]);
                }
                m_local_goal_pose_initialized = true;
                return true;
            }

            int IndexOffset = 0;
            if (m_local_goal_pose_index >= mIndexOffsetLocalGoalPose)
            {
                IndexOffset = mIndexOffsetLocalGoalPose / 2;
            }
            double dist = Decision::caculate_distance(m_global_path.poses[m_local_goal_pose_index - IndexOffset].pose, m_local_start_pose.pose);
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
                    
                    for (size_t i = 0; i < mIndexOffsetLocalGoalPose + IndexOffset * 2; i++)
                    {
                        m_local_path.poses.push_back(m_global_path.poses[m_local_goal_pose_index + i - IndexOffset * 2]);
                    }
                    m_local_goal_pose_index += mIndexOffsetLocalGoalPose;
                }
            }
            m_local_goal_pose.pose = m_global_path.poses[m_local_goal_pose_index].pose;
            m_local_goal_pose.header.frame_id = "map";
            m_local_goal_pose.header.stamp = ros::Time::now();
            m_local_goal_pose.pose.orientation.w = 1;
            std::cout << "m_local_goal_pose = \n"
                      << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;
        }
        return true;

        // boost::lock_guard<boost::mutex> lock_guard(m_global_path_mutex);
        len = m_global_path.poses.size();
        std::cout << "The m_global_path len = " << len << std::endl;
        if (len < 1)
        {
            return false;
        }
        if (len < 10 )
        {
            std::cout << "The  m_global_path.poses[" << len - 1 << "] = " << m_global_path.poses[len - 1] << endl;
            m_local_goal_pose = m_global_path.poses[len - 1];
            std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;
        }
        else
        {
            if (Decision::caculate_distance(m_global_path.poses[m_local_goal_pose_index].pose, m_local_start_pose.pose) < mSetDistLocalStartToGoal)
            {
                m_local_goal_pose_index += mIndexOffsetLocalGoalPose;
                if (m_local_goal_pose_index >= len)
                {
                    m_local_goal_pose_index = 10;
                }
            }
            m_local_goal_pose.pose = m_global_path.poses[m_local_goal_pose_index].pose;
            m_local_goal_pose.header.frame_id = "map";
            m_local_goal_pose.header.stamp = ros::Time::now();
            m_local_goal_pose.pose.orientation.w = 1;
            std::cout << "m_local_goal_pose = \n"
                      << m_local_goal_pose << std::endl;
            m_local_goal_pose_initialized = true;
            return true;
        }
    }

    void local_plan_thread()
    {
        ROS_INFO("The Decision::local_plan_thread 被执行 ..."); //local_map
        m_local_map_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>("/ow/local_map", 1, &Decision::local_map_cb, this);
        m_local_start_pose_sub = m_nh.subscribe("/initialpose", 1, &Decision::local_start_pose_cb, this); //TODO 只留一个
        m_local_goal_pose_sub = m_nh.subscribe("move_base_simple/goal", 1, &Decision::local_goal_pose_cb, this);
        
        m_local_raw_path_pub = m_nh.advertise<nav_msgs::Path>("local_raw_path", 1);
        m_local_path_pub = m_nh.advertise<nav_msgs::Path>("/ow/local_path", 1);
        m_system_command_pub = m_nh.advertise<pkg_nav::system_command>("/ow/system_command", 10);  //
        mOdometryPub = m_nh.advertise<nav_msgs::Odometry>("/ow/odometry", 1);
        ROS_WARN("The local_plan_thread 已经启动，请设置局部地图及起始点....");

        pkg_nav::local_pathGoal goal; //定义要做的目标
        pkg_nav::global_path::Request request;
        pkg_nav::global_path::Response response;
        unsigned count = 1;
        unsigned index = 0;
        ros::Rate loop_rate(5);
        while (ros::ok)
        {
            // int run_state = m_run_state;
            m_system_command.runCommand = m_run_state;
            m_system_command_pub.publish(m_system_command);
            
            if (m_run_state != E_SystemRunState::IN_planning)
            {
                loop_rate.sleep();
                continue;
            }
            ros::spinOnce();
            mActDistLocalStartToGoal = caculate_distance(m_local_start_pose.pose, m_global_goal_pose.pose);
            if (local_path_update())
            {
                // m_local_start_pose_initialized = false;
                if (mPathShae == E_PathShape::AStar)
                {
                    m_local_goal_pose_initialized = Decision::LineMotionLocal();//TODO
                    // m_local_goal_pose_initialized = Decision::get_local_goal_pose();
                }
                else
                {
                    m_local_goal_pose_initialized = Decision::LineMotionLocal();
                }
               
                if (!m_local_goal_pose_initialized)
                {
                    ROS_WARN("局部目标位姿m_local_goal_pose_initialized 还未初始化....");
                    loop_rate.sleep();
                    continue;
                }
                request.num = 10;
                request.start_pose = m_local_start_pose;
                request.goal_pose = m_local_goal_pose;
                if (mPathShae == E_PathShape::AStar)
                {
                    if (mActDistLocalStartToGoal > mSetDistLocalStartToGoal / 3)
                    {
                        m_local_path_initialized = m_local_goal_pose_initialized;
                        // m_local_path_initialized = Decision::local_plan_response(request, response);
                    }
                }
                else
                {
                    m_local_path_initialized = m_local_goal_pose_initialized;
                }
                if (m_local_path_initialized)
                {
                    if (mPathShae == E_PathShape::AStar)
                    {
                        ;
                        // m_local_path = response.global_path;
                    }
                    m_local_path.header.frame_id = "map";         
                    m_local_path.header.seq = count;              // TODO
                    m_local_path.header.stamp = ros::Time::now(); // TODO
                    m_local_raw_path.header.frame_id = "map";
                    m_local_raw_path.header.stamp = ros::Time::now();
                    m_local_raw_path_pub.publish(m_local_raw_path);

                    //======mOdometry msg===
                    // mOdometry.header.frame_id = "map"; 
                    // if (index >= m_local_path.poses.size() - 1)
                    // {
                    //     index = m_local_path.poses.size() - 1;
                    // }
                    // mOdometry.pose.pose = m_local_path.poses[index].pose;
                    // index += 1;
                    // mOdometryPub.publish(mOdometry);
                    // int colorRGB[3] = {0, 1, 0};
                    // Display(mOdometry.pose.pose, colorRGB);

                    m_local_path_pub.publish(m_local_path); //ow/local_path for test
                    ROS_ERROR("The m_local_path_pub count = %d", count++);

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
                    // Decision::get_start_pose(); TODO
                    // m_local_start_pose_initialized = false;
                }
                else
                {
                    ROS_FATAL("The local_plan_response 执行失败，正在尝试重新规划");
                    m_local_goal_pose_index += 25;
                    if (m_local_goal_pose_index > 200)
                    {
                        ROS_FATAL("The local_plan_response 执行失败....");
                        m_err_state = E_SystemErrorState::Err_local_planning;
                        m_run_state = E_SystemRunState::IN_emergency_stop;
                        m_local_path.poses.clear();
                        m_local_path_pub.publish(m_local_path);
                        return;
                    }
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
        int num = request.num;
        int sum = 0;
        ROS_INFO("The local_plan_response 正在处理数据... ,request.num = %d", num);
        if (num < 0)
        {
            ROS_FATAL("提交的数据异常：数据不可以为负数 ！");
            return false;
        }

        ros::NodeHandle nh;
        ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("a_star_path", 1);
        ros::Publisher circle_pub = nh.advertise<visualization_msgs::MarkerArray>("a_star_circle", 1);

        geometry_msgs::Pose start_pose, goal_pose;
        bool is_point_robot = false; // 是否为点状机器人
        // 可视化非常耗费时间, 若不需要查看A*算法中的探索过程,
        // 建议将可视化的标志位置为false
        bool is_visualization = false;  // 是否需要可视化中间过程
        bool is_eight_connected = true; // 每个节点的方向是否为八连通
        std::unique_ptr<AStar> a_star_ptr = std::make_unique<AStar>(nh, is_visualization, is_eight_connected);
        AStarResult a_star_result;

        nav_msgs::Path path;
        int id = 0;

        // a_star_ptr->SetMap(m_local_map); //TODO
        a_star_ptr->SetMap(m_global_map); //TODO
        if (!is_point_robot)
        {
            // 如果不是点机器人, 则需要设置长宽等参数
            a_star_ptr->SetVhicleParams(5.0, 5.0, 3.0);
        }
        ros::Time start_time = ros::Time::now();
        start_pose = request.start_pose.pose;
        goal_pose = request.goal_pose.pose;
        if (a_star_ptr->SearchPath(start_pose.position.x, start_pose.position.y, goal_pose.position.x, goal_pose.position.y, &a_star_result))
        {
            float delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[查找local path delta_time =  " << delta_time << " s]!" << std::endl;
            size_t len = a_star_result.x.size();
            int path_smooth_methed = 2;
            geometry_msgs::PoseStamped pose;
            if (1 == path_smooth_methed)
            {
                for (size_t i = 0; i < len; i++)
                {
                    pose.pose.position.x = a_star_result.x[i];
                    pose.pose.position.y = a_star_result.y[i];
                    path.poses.push_back(std::move(pose));
                }
            }
            else if (1 == path_smooth_methed)
            {
                nav_msgs::Path raw_path;
                geometry_msgs::PoseStamped raw_pose;
                for (int i = 0; i < len; ++i)
                {
                    raw_pose.pose.position.x = a_star_result.x[i];
                    raw_pose.pose.position.y = a_star_result.y[i];
                    raw_path.poses.push_back(raw_pose);
                }
                m_local_raw_path = raw_path;
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
                geometry_msgs::PoseStamped pose;
                len = smooth_point3d.size();
                for (size_t i = 0; i < len; i++)
                {
                    pose.pose.position = smooth_point3d[i];
                    path.poses.push_back(std::move(pose));
                }
            }
            else if (2 == path_smooth_methed) //quadratic programming
            {
                std::cout << "OSQP RUNNING..." << std::endl;
                std::vector<std::pair<double, double>> raw_point2d;
                std::vector<double> bounds;
                std::vector<std::pair<double, double>> upperBounds;
                std::vector<std::pair<double, double>> lowerBounds;
                geometry_msgs::PoseStamped raw_pose;
                nav_msgs::Path raw_path;
                for (int i = 0; i < len; ++i)
                {
                    raw_pose.pose.position.x = a_star_result.x[i];
                    raw_pose.pose.position.y = a_star_result.y[i];
                    raw_path.poses.push_back(raw_pose);
                    raw_point2d.push_back(std::make_pair(a_star_result.x[i], a_star_result.y[i]));
                    upperBounds.push_back(std::make_pair(1, 1));
                    lowerBounds.push_back(std::make_pair(1, 1));
                    // bounds.push_back(1);
                }
                upperBounds[0].first = 0.1;
                upperBounds[0].second = 0.1;
                lowerBounds[len - 1].first = 0.1;
                lowerBounds[len - 1].second = 0.1;
                m_local_raw_path = raw_path;

                FemPosDeviationSqpOsqpInterface solver;
                solver.set_ref_points(raw_point2d);
                solver.set_bounds_around_refs(upperBounds, lowerBounds);
                if (!solver.Solve())
                {
                    std::cout << "failture" << std::endl;
                    ROS_FATAL("The FemPosDeviationSqpOsqpInterface solver is  failture !");
                }
                std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();
                std::cout << "OSQP END!!!" << std::endl;
                for (size_t i = 0; i < len; i++)
                {
                    pose.pose.position.x = opt_xy[i].first;
                    pose.pose.position.y = opt_xy[i].second;
                    path.poses.push_back(std::move(pose));
                }
            }
            delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[局部路径总时间 local path delta_time =  " << delta_time << " s]!" << std::endl;
        }
        else
        {
            ROS_FATAL("局部路径查找失败 ！");
            return false;
        }

        for (int i = 1; i <= num; i++)
        {
            sum += i;
        }
        response.sum = sum;
        response.global_path = path;
        ROS_WARN("The test for local_plan_response of response.global_path is normal");
        return true;
    }

    //处理最终结果
    void done_cb(const actionlib::SimpleClientGoalState &state, const pkg_nav::local_pathResultConstPtr &result)
    {
        if (state.state_ == state.SUCCEEDED)
        {
            ROS_INFO("The local_pathAction server 执行成功,result = %d", result->result);
            // m_local_path_initialized = true;
        }
        else
        {
            ROS_INFO("The local_pathAction server 任务失败！");
            ;
            //     ROS_INFO("Cancel Goal!");
            //     m_local_path_client.cancelAllGoals();
        }
    }

    void active_cb() // 当目标激活的时候，会调用一次
    {
        // return;
        ROS_WARN("The local_pathAction server 已经被激活....");
        // ROS_WARN("The test local_pathAction is normal");
    }

    //接收服务器连续反馈信息
    void feedback_cb(const pkg_nav::local_pathFeedbackConstPtr &feedback)
    {
        // return;
        ROS_INFO("当前进度:%.2f", feedback->progress_bar);
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

    float caculate_distance(geometry_msgs::Pose pos1, geometry_msgs::Pose pos2)
    {
        float dx = pos1.position.x - pos2.position.x;
        float dy = pos1.position.y - pos2.position.y;
        float dz = pos1.position.z - pos2.position.z;
        return sqrt(dx * dx + dy * dy + dz * dz);
    }

    // bool local_path_process
    VectorXd polynomial_fitting(VectorXd v_x, VectorXd v_y, const int m = 3) //polynomial_dimension
    {
        int len = v_x.size();
        MatrixXd A(m + 1, m + 1);
        VectorXd b(m + 1);
        for (int i = 0; i < m + 1; i++)
        {
            VectorXd temp(len);
            for (int k = 0; k < len; k++)
            {
                temp(k) = pow(v_x(k), i);
            }
            b(i) = v_y.dot(temp);
            for (int j = 0; j < m + 1; j++)
            {
                for (int k = 0; k < len; k++)
                {
                    temp(k) = pow(v_x(k), i + j);
                }
                A(i, j) = temp.sum();
            }
        }
        // std::cout << "b=\n"
        //           << b << std::endl;
        // std::cout << "A=\n"
        //           << A << std::endl;
        // VectorXd x = A.colPivHouseholderQr().solve(b);
        // std::cout << "v_a=\n"
        //           << v_a << std::endl;
        b = A.colPivHouseholderQr().solve(b);
        return b;
    }

    void SplinePlanning(float *x, float *y, int count, float *a0, float *a1, float *a2, float *a3, float begin_dy1 = 0, float end_dy1 = 0)
    {
        //分段函数的形式为 Si(x) =  a0 + a1(x-xi) + a2(x-xi)^2 + a3(x-xi)^3
        //xi为x[i]的值，xi_1为x[i+1]的值
        // float a0[count];
        // float a1[count];
        // float a2[count];
        // float a3[count];
        //中间变量
        float h[count];
        float fi[count];
        float u[count];
        float a[count];
        float d[count];
        float m[count];
        float b[count];
        float yy[count];
        float dy1[count];
        float dy2[count];
        //求h和fi
        for (size_t i = 0; i < count - 2; i++)
        {
            h[i] = x[i + 1] - x[i];
            fi[i] = (y[i + 1] - y[i]) / h[i];
        }
        for (size_t i = 1; i < count - 2; i++)
        {
            u[i] = h[i - 1] / (h[i - 1] + h[i]); //mu
            a[i] = h[i] / (h[i - 1] + h[i]);     //lambda ^A
            d[i] = 6 * (fi[i] - fi[i - 1]) / (h[i - 1] + h[i]);
        }

        //计算边界条件
        u[count - 1] = 1; //h[n] = 0
        a[0] = 1;         //h[0]=0
        d[0] = 6 * (fi[0] - begin_dy1) / h[0];
        d[count - 1] = 6 * (end_dy1 - fi[count - 2]) / h[count - 2];

        //追赶法求解M矩阵
        b[0] = a[0] / 2;
        for (size_t i = 1; i < count - 2; i++)
        {
            b[i] = a[i] / (2 - u[i] * b[i - 1]);
        }

        yy[0] = d[0] / 2;
        for (size_t i = 1; i < count - 1; i++)
        {
            yy[i] = (d[i] - u[i] * yy[i - 1]) / (2 - u[i] * b[i - 1]);
        }

        m[count - 1] = yy[count - 1];
        for (size_t i = count - 1; i > 0; i--)
        {
            m[i - 1] = yy[i - 1] - b[i - 1] * m[i];
        }

        //计算方程最终结果
        std::cout << "//计算方程最终结果" << std::endl;
        for (size_t i = 0; i < count - 2; i++)
        {
            a0[i] = y[i];
            a1[i] = fi[i] - h[i] * m[i] / 2 - h[i] * (m[i + 1] - m[i]) / 6;
            a2[i] = m[i] / 2;
            a3[i] = (m[i + 1] - m[i]) / (6 * h[i]);
            std::cout << a0[i] << std::endl;
            std::cout << a1[i] << std::endl;
            std::cout << a2[i] << std::endl;
            std::cout << a3[i] << std::endl;
        }

        dy1[0] = begin_dy1;
        dy1[count - 1] = end_dy1;
        dy2[0] = 2 * a2[0];
        for (size_t i = 1; i < count - 2; i++)
        {
            dy1[i] = a1[i - 1] + 2 * a2[i - 1] * h[i - 1] + 3 * a3[i - 1] * h[i - 1] * h[i - 1];
            dy2[i] = 2 * a2[i - 1] + 6 * a3[i - 1] * h[i - 1];
        }
        dy2[count - 1] = 2 * a2[count - 2] + 6 * a3[count - 2] * h[count - 2];
    }

    void Display(geometry_msgs::Pose pose, int colorRGB[])
    {
        //实例化一个Marker
        visualization_msgs::Marker mMarker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        // 设置frame ID 和 时间戳
        mMarker.header.frame_id = "map";
        mMarker.header.stamp = ros::Time::now();
        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        // 为这个marker设置一个独一无二的ID，一个marker接收到相同ns和id就会用新的信息代替旧的
        mMarker.ns = "basic_shapes";
        mMarker.id = 0;
        // Set the mMarker type.  Initially this is sphere, and cycles between that and SPHERE, ARROW, and CYLINDER
        mMarker.type = mShape;
        // Set the mMarker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        mMarker.action = visualization_msgs::Marker::ADD;
        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        // 设置marker的位置
        mMarker.pose.position.x = pose.position.x;
        mMarker.pose.position.y = pose.position.y;
        mMarker.pose.position.z = 0;
        mMarker.pose.orientation.x = 0.0;
        mMarker.pose.orientation.y = 0.0;
        mMarker.pose.orientation.z = 0.0;
        mMarker.pose.orientation.w = 1.0;
        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        // 设置marker的大小
        mMarker.scale.x = 5;
        mMarker.scale.y = 5;
        mMarker.scale.z = 5;
        // Set the color -- be sure to set alpha to something non-zero!
        // 设置marker的颜色
        mMarker.color.r = colorRGB[0]; // 红色（R）0 到 255 间的整数，代表颜色中的红色成分
        mMarker.color.g = colorRGB[1]; //绿色（G）0 到 255 间的整数，代表颜色中的绿色成分。
        mMarker.color.b = colorRGB[2]; //蓝色（B）0 到 255 间的整数，代表颜色中的蓝色成分。
        mMarker.color.a = 1.0;         // 透明度（A）取值 0~1 之间， 代表透明度。
        //取消自动删除
        mMarker.lifetime = ros::Duration();
        // Publish the marker
        // 必须有订阅者才会发布消息
        // while (markerPub.getNumSubscribers() < 1)
        // {
        //     if (!ros::ok())
        //     {
        //         return;
        //     }
        //     ROS_WARN_ONCE("Please create a subscriber to the marker");
        //     sleep(1);
        // }
        mMarkerPub.publish(mMarker);
    }
};

class AstarNavi
{
private:
    // ros
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher m_global_path_pub; //Publisher global path TODO
    nav_msgs::Path m_global_path;
    ros::Publisher lane_pub_;
    ros::Subscriber costmap_sub_;
    ros::Subscriber current_pose_sub_;
    ros::Subscriber goal_pose_sub_;
    tf::TransformListener tf_listener_;

    // params
    float waypoints_velocity_; // constant velocity on planned waypoints [km/h]
    float update_rate_;        // replanning and publishing rate [Hz]

    // classes
    AstarSearch astar_;

    // variables
    nav_msgs::OccupancyGrid costmap_;
    geometry_msgs::PoseStamped current_pose_local_, current_pose_global_;
    geometry_msgs::PoseStamped goal_pose_local_, goal_pose_global_;
    tf::Transform local2costmap_; // local frame (e.g. velodyne) -> costmap origin

    bool costmap_initialized_;
    bool current_pose_initialized_;
    bool goal_pose_initialized_;

public:
    AstarNavi() : nh_(), private_nh_("~")
    {
        ROS_INFO("AstarNavi::AstarNavi() 被执行");
        private_nh_.param<float>("waypoints_velocity", waypoints_velocity_, 5.0);
        private_nh_.param<float>("update_rate", update_rate_, 1.0);

        lane_pub_ = nh_.advertise<autoware_msgs::LaneArray>("lane_waypoints_array", 1, true);
        m_global_path_pub = nh_.advertise<nav_msgs::Path>("/ow/global_path", 1); // TODO
        // costmap_sub_ = nh_.subscribe("costmap", 1, &AstarNavi::costmapCallback, this); //cyb
        costmap_sub_ = nh_.subscribe("/ow/global_map", 1, &AstarNavi::costmapCallback, this); //TODO
        // current_pose_sub_ = nh_.subscribe("current_pose", 1, &AstarNavi::currentPoseCallback, this);//cyb
        current_pose_sub_ = nh_.subscribe("/initialpose", 1, &AstarNavi::currentPoseCallback, this); //TODO
        //ros::Subscriber start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
        goal_pose_sub_ = nh_.subscribe("move_base_simple/goal", 1, &AstarNavi::goalPoseCallback, this);

        costmap_initialized_ = false;
        current_pose_initialized_ = false;
        goal_pose_initialized_ = false;
    }
    ~AstarNavi()
    {
    }
    void run()
    {
        ros::Rate rate(update_rate_);
        nav_msgs::Path empty_path;
        empty_path.header.stamp = ros::Time::now();
        empty_path.header.frame_id = costmap_.header.frame_id;

        while (ros::ok())
        {
            ros::spinOnce();

            if (!costmap_initialized_ || !current_pose_initialized_ || !goal_pose_initialized_)
            {
                rate.sleep();
                continue;
            }

            // initialize vector for A* search, this runs only once
            astar_.initialize(costmap_);

            // update local goal pose
            goalPoseCallback(goal_pose_global_);

            // execute astar search
            ros::WallTime start = ros::WallTime::now();
            bool result = astar_.makePlan(current_pose_local_.pose, goal_pose_local_.pose);
            ros::WallTime end = ros::WallTime::now();

            ROS_INFO("Astar planning: %f [s]", (end - start).toSec());

            if (result)
            {
                ROS_INFO("全局路查找成功！");
                publishWaypoints(astar_.getPath(), waypoints_velocity_);
            }
            else
            {
                ROS_INFO("全局路查找失败！");
                publishStopWaypoints();
            }

            astar_.reset();
            rate.sleep();
        }
    }

private:
    // functions, callback
    void costmapCallback(const nav_msgs::OccupancyGrid &msg)
    {
        ROS_INFO("The global_map 高*宽 = %d*%d", msg.info.height, msg.info.width);
        costmap_ = msg;
        tf::poseMsgToTF(costmap_.info.origin, local2costmap_);
        costmap_initialized_ = true;
    }

    void currentPoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
    {
        if (!costmap_initialized_)
        {
            return;
        }
        ROS_INFO("currentPose x = %f, y = %f", msg.pose.pose.position.x, msg.pose.pose.position.y);
        current_pose_global_.header = msg.header;  //TODO
        current_pose_global_.pose = msg.pose.pose; //TODO
        // current_pose_global_ = msg; //cyb
        current_pose_local_.pose = transformPose(
            current_pose_global_.pose, getTransform(costmap_.header.frame_id, current_pose_global_.header.frame_id));
        current_pose_local_.header.frame_id = costmap_.header.frame_id;
        current_pose_local_.header.stamp = current_pose_global_.header.stamp;
        current_pose_initialized_ = true;
    }

    void goalPoseCallback(const geometry_msgs::PoseStamped &msg)
    {
        if (!costmap_initialized_)
        {
            return;
        }
        ROS_INFO("goalPose x = %f, y = %f", msg.pose.position.x, msg.pose.position.y);

        goal_pose_global_ = msg;
        goal_pose_local_.pose =
            transformPose(goal_pose_global_.pose, getTransform(costmap_.header.frame_id, goal_pose_global_.header.frame_id));
        goal_pose_local_.header.frame_id = costmap_.header.frame_id;
        goal_pose_local_.header.stamp = goal_pose_global_.header.stamp;

        goal_pose_initialized_ = true;

        ROS_INFO_STREAM("Subscribed goal pose and transform from " << msg.header.frame_id << " to "
                                                                   << goal_pose_local_.header.frame_id << "\n"
                                                                   << goal_pose_local_.pose);
    }

    // fucntions
    tf::Transform getTransform(const std::string &from, const std::string &to)
    {
        tf::StampedTransform stf;
        try
        {
            tf_listener_.lookupTransform(from, to, ros::Time(0), stf);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        return stf;
    }

    void publishWaypoints(const nav_msgs::Path &path, const float &velocity)
    {
        autoware_msgs::Lane lane;
        lane.header.frame_id = "map";
        lane.header.stamp = path.header.stamp;
        lane.increment = 0;

        for (const auto &pose : path.poses)
        {
            autoware_msgs::Waypoint wp;
            wp.pose.header = lane.header;
            wp.pose.pose = transformPose(pose.pose, getTransform(lane.header.frame_id, pose.header.frame_id));
            wp.pose.pose.position.z = current_pose_global_.pose.position.z; // height = const
            wp.twist.twist.linear.x = velocity / 3.6;                       // velocity = const
            lane.waypoints.push_back(wp);
        }

        autoware_msgs::LaneArray lane_array;
        lane_array.lanes.push_back(lane);
        lane_pub_.publish(lane_array);

        m_global_path = path;                     //TODO
        m_global_path.header.frame_id = "map";    //TODO
        m_global_path_pub.publish(m_global_path); //TODO
        ROS_INFO("全局路径已发布 ！");
    }

    void publishStopWaypoints()
    {
        nav_msgs::Path path;
        geometry_msgs::PoseStamped pose; // stop path
        pose.header.stamp = ros::Time::now();
        pose.header.frame_id = current_pose_global_.header.frame_id;
        pose.pose = current_pose_global_.pose;
        path.poses.push_back(pose);
        publishWaypoints(path, 0.0);
        ROS_INFO("全局路径已发布,但是查找失败 ！");

        m_global_path = path;                     //TODO
        m_global_path.header.frame_id = "map";    //TODO
        m_global_path_pub.publish(m_global_path); //TODO
    }
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
    return 0;
}
