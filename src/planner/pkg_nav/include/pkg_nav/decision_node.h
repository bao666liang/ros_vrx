#ifndef DECISION_NODE_H
#define DECISION_NODE_H
#include <iostream>
#include <vector>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <autoware_msgs/LaneArray.h>
#include "pkg_nav/global_paln/astar_search.h"
#include <eigen3/Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

/// 系统运行状态
namespace E_SystemRunState
{
    enum E_SystemRunState
    {
        IN_idle,            //0
        IN_initializing,    //初始化
        IN_waiting_command, //2
        IN_planning,        //3
        IN_controlling,     //在控制机器人运动中
        IN_normal_stop,     //5
        IN_emergency_stop,  //6
        IN_reset            //复位
    };
};

// 出错恢复行为
namespace E_SystemErrorState
{
    enum E_SystemErrorState
    {
        Err_ok,
        Err_global_planning, //全局规划失败
        Err_local_planning,  //局部规划失败
        Err_controlling,     //控制失败
        Err_oscillation      //长时间困在一片小区域
    };
};

// pathshap
namespace E_PathShape
{
    enum E_PathShape
    {
        AStar,        //0
        Line,         //1
        Square,       //2
        Circle,       //3
        CircleDouble, //4
        CircleS,       //5
        SpiralLine //6
    };
};


namespace DC
{
    // void local_goal_pose_cb(const geometry_msgs::PoseStamped &local_goal_pose)
    // {
    //     if (!m_local_map_initialized)
    //     {
    //         return;
    //     }
    //     m_local_path.poses.clear();
    //     m_local_path_initialized = false;
    //     m_local_goal_pose_initialized = false;
    //     m_local_goal_pose_index = 100;
    //     // m_local_goal_pose = local_goal_pose;
    //     // m_local_goal_pose_initialized = true;
    //     // std::cout << "m_local_goal_pose = " << m_local_goal_pose << std::endl;
    //     // ROS_INFO("The m_local_goal_pose x = %f, y = %f", m_local_goal_pose.pose.position.x, m_local_goal_pose.pose.position.y);
    // }

    //处理最终结果
    // void done_cb(const actionlib::SimpleClientGoalState &state, const pkg_nav::local_pathResultConstPtr &result)
    // {
    //     if (state.state_ == state.SUCCEEDED)
    //     {
    //         ROS_INFO("The local_pathAction server 执行成功,result = %d", result->result);
    //         // m_local_path_initialized = true;
    //     }
    //     else
    //     {
    //         ROS_INFO("The local_pathAction server 任务失败！");
    //         //     ROS_INFO("Cancel Goal!");
    //         //     m_local_path_client.cancelAllGoals();
    //     }
    // }

    // void active_cb() // 当目标激活的时候，会调用一次
    // {
    //     // return;
    //     ROS_WARN("The local_pathAction server 已经被激活....");
    //     // ROS_WARN("The test local_pathAction is normal");
    // }

    //接收服务器连续反馈信息
    // void feedback_cb(const pkg_nav::local_pathFeedbackConstPtr &feedback)
    // {
    //     // return;
    //     ROS_INFO("当前进度:%.2f", feedback->progress_bar);
    // }

    // bool RefLineSmooth(std::vector<std::pair<double, double>> raw_point2d, std::vector<std::pair<double, double>> lowerBounds, std::vector<std::pair<double, double>> upperBounds, std::vector<std::pair<double, double>> &ref_point2d)
    // {
    //     FemPosDeviationSqpOsqpInterface solver;
    //     std::cout << "OSQP RUNNING..." << std::endl;
    //     solver.set_ref_points(raw_point2d);
    //     solver.set_bounds_around_refs(upperBounds, lowerBounds);
    //     if (!solver.Solve())
    //     {
    //         std::cout << "failture" << std::endl;
    //         ROS_FATAL("The FemPosDeviationSqpOsqpInterface solver is  failture !");
    //         return false;
    //     }
    //     ref_point2d = solver.opt_xy();
    //     std::cout << "OSQP END!!!" << std::endl;
    //     return true;
    // }
};

namespace Bezuer
{
    class Bezuer_curve
    {
    public:
        Bezuer_curve(std::vector<geometry_msgs::Point> &raw_point3d, std::vector<geometry_msgs::Point> &smooth_point3d, unsigned interval_count = 4)
        {
            if (raw_point3d.size() == 0)
                return;
            smooth_point3d.clear();
            // geometry_msgs::Point a_point;
            // geometry_msgs::Point b_point;
            // geometry_msgs::Point c_point;
            // geometry_msgs::Point d_point;
            std::vector<float>
                v_t; //插值向量
            // a_point.x = 0;
            // a_point.y = 0;
            // b_point.x = 2;
            // b_point.y = 2;
            // c_point.x = 4;
            // c_point.y = 0;
            // d_point.x = 6;
            // d_point.y = 2;
            // raw_point3d.push_back(a_point);
            // raw_point3d.push_back(b_point);
            // raw_point3d.push_back(c_point);
            // raw_point3d.push_back(d_point);
            if (!linespace(0, 1, interval_count, v_t))
            {
                return;
            }
            // for (auto t : v_t)
            // {
            //     std::cout << " t = " << t << std::endl;
            // }
            gen_bezuer_path(raw_point3d, v_t, smooth_point3d);
            // std::cout << "smooth_point3d = \n";
            // for (auto point : smooth_point3d)
            // {
            //     std::cout << " point = \n"
            //               << point << std::endl;
            // }
        }

        //生成单个点的控制点
        void gen_single_control_dot(geometry_msgs::Point a_point, geometry_msgs::Point b_point, geometry_msgs::Point c_point, geometry_msgs::Point &e1_point, geometry_msgs::Point &f1_point)
        {
            geometry_msgs::Point e_point;
            geometry_msgs::Point f_point;
            geometry_msgs::Point d_point;

            e_point.x = (a_point.x + b_point.x) / 2;
            e_point.y = (a_point.y + b_point.y) / 2;
            f_point.x = (b_point.x + c_point.x) / 2;
            f_point.y = (b_point.y + c_point.y) / 2;

            float l_ab = caculate_point_distance(a_point, b_point);
            float l_bc = caculate_point_distance(b_point, c_point);
            float k = l_ab / l_bc;
            float t = k / (1 + k);
            //D = f + (e - f) * lcb / (lcb + lab)
            d_point.x = (1 - t) * e_point.x + t * f_point.x;
            d_point.y = (1 - t) * e_point.y + t * f_point.y;
            geometry_msgs::Point db_point;
            db_point.x = b_point.x - d_point.x;
            db_point.y = b_point.y - d_point.y;
            e1_point.x = e_point.x + db_point.x;
            e1_point.y = e_point.y + db_point.y;
            f1_point.x = f_point.x + db_point.x;
            f1_point.y = f_point.y + db_point.y;
            // std::cout << "e_point = \n"
            //           << e_point << "f_point =\n"
            //           << f_point << endl;
            // std::cout << "d_point = \n"
            //           << d_point << "db_point = \n"
            //           << db_point << endl;
            // std::cout << "e1_point = \n"
            //           << e1_point << "f1_point = \n"
            //           << f1_point << endl;
        }
        // 生成全部点的控制点
        bool gen_all_control_dot(std::vector<geometry_msgs::Point> &raw_point3d, std::vector<geometry_msgs::Point> &all_control_point)
        {
            geometry_msgs::Point a_point = raw_point3d[0];
            geometry_msgs::Point b_point = raw_point3d[1];
            geometry_msgs::Point c_point = raw_point3d[2];
            geometry_msgs::Point e1_point;
            geometry_msgs::Point f1_point;
            gen_single_control_dot(a_point, b_point, c_point, e1_point, f1_point);
            geometry_msgs::Point temp_point;
            temp_point.x = (a_point.x + e1_point.x) / 2;
            temp_point.y = (a_point.y + e1_point.y) / 2;
            all_control_point.push_back(a_point);
            all_control_point.push_back(temp_point);

            int count = raw_point3d.size();
            for (int i = 0; i < count - 2; i++)
            {
                gen_single_control_dot(raw_point3d[i], raw_point3d[i + 1], raw_point3d[i + 2], e1_point, f1_point);
                all_control_point.push_back(e1_point);
                all_control_point.push_back(raw_point3d[i + 1]);
                all_control_point.push_back(f1_point);
            }
            temp_point.x = (f1_point.x + raw_point3d[count - 1].x) / 2;
            temp_point.y = (f1_point.y + raw_point3d[count - 1].y) / 2;
            all_control_point.push_back(temp_point);
            all_control_point.push_back(raw_point3d[count - 1]);
        }
        //规划两点之间的路径（三阶贝塞尔曲线）
        void gen_bezier_curve(std::vector<geometry_msgs::Point> control_point, std::vector<float> v_t, std::vector<geometry_msgs::Point> &smooth_point3d)
        {
            int count = v_t.size();
            geometry_msgs::Point point;
            for (int i = 0; i < count; i++)
            {
                float t = v_t[i];
                point.x = control_point[0].x * (1 - t) * (1 - t) * (1 - t) + 3 * control_point[1].x * t * (1 - t) * (1 - t) + 3 * control_point[2].x * t * t * (1 - t) + control_point[3].x * t * t * t;
                point.y = control_point[0].y * (1 - t) * (1 - t) * (1 - t) + 3 * control_point[1].y * t * (1 - t) * (1 - t) + 3 * control_point[2].y * t * t * (1 - t) + control_point[3].y * t * t * t;
                smooth_point3d.push_back(point);
            }
        }
        // 规划所有点之间的路径
        void gen_bezuer_path(std::vector<geometry_msgs::Point> &raw_point3d, std::vector<float> v_t, std::vector<geometry_msgs::Point> &smooth_point3d)
        {
            std::vector<geometry_msgs::Point> all_control_point;
            std::vector<geometry_msgs::Point> control_point(4);
            gen_all_control_dot(raw_point3d, all_control_point);
            // for (auto point : all_control_point)
            // {
            //     std::cout << " point = \n"
            //               << point << std::endl;
            // }
            // for (int i = 0; i < 4; i++)
            // {
            //     control_point[i] = all_control_point[i];
            // }

            int count = raw_point3d.size();
            if (count == 2)
            {
                count = 1;
            }
            else
            {
                count = count / 2 + 1;
            }
            for (int i = 0; i < count; i++)
            {
                control_point[0] = all_control_point[3 * i];
                control_point[1] = all_control_point[3 * i + 1];
                control_point[2] = all_control_point[3 * i + 2];
                control_point[3] = all_control_point[3 * i + 3];
                gen_bezier_curve(control_point, v_t, smooth_point3d);
            }
        }

        float caculate_point_distance(geometry_msgs::Point a_point, geometry_msgs::Point b_point)
        {
            return std::sqrt((b_point.x - a_point.x) * (b_point.x - a_point.x) + (b_point.y - a_point.y) * (b_point.y - a_point.y) + (b_point.z - a_point.z) * (b_point.z - a_point.z));
        }
        bool linespace(float start, float end, unsigned count, std::vector<float> &v_t)
        {
            if (count < 2)
            {
                std::cout << "The input count is zero !" << std::endl;
                return false;
            }
            if (count == 2)
            {
                v_t.push_back(start);
                v_t.push_back(end);
                return true;
            }
            float dt = (end - start) / (count - 1);
            for (int i = 0; i < count; i++)
            {
                v_t.push_back(start + i * dt);
            }
            return true;
        }
    };
};


namespace AutoWare
{
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
}



#endif