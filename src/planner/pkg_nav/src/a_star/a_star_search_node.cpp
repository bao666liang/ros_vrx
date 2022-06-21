#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <visualization_msgs/MarkerArray.h>
#include "pkg_nav/global_paln/a_star_search.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "a_star_search_node");

    ros::NodeHandle nh;
    ros::Publisher path_pub = nh.advertise<nav_msgs::Path>("a_star_path", 1);
    ros::Publisher circle_pub = nh.advertise<visualization_msgs::MarkerArray>("a_star_circle", 1);

    geometry_msgs::Pose start_pose, goal_pose;
    nav_msgs::OccupancyGrid grid_map;
    bool is_start_set = false;
    bool is_goal_set = false;
    bool has_grid_map = false;
    bool is_point_robot = true; // 是否为点状机器人
    // 可视化非常耗费时间, 若不需要查看A*算法中的探索过程,
    // 建议将可视化的标志位置为false
    bool is_visualization = true;  // 是否需要可视化中间过程
    bool is_eight_connected = true; // 每个节点的方向是否为八连通

    // 应该订阅无人车的位姿, 此处仅用来测试算法,
    // 可根据实际情况修改订阅的消息类型
    ros::Subscriber start_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1,
                                                                                       [&](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
                                                                                       {
                                                                                           ROS_INFO("currentPose x = %f, y = %f", msg->pose.pose.position.x, msg->pose.pose.position.y);
                                                                                           start_pose = msg->pose.pose;
                                                                                           is_start_set = true;
                                                                                       });

    ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1,
                                                                        [&](const geometry_msgs::PoseStamped::ConstPtr &msg)
                                                                        {
                                                                            ROS_INFO("goalPose x = %f, y = %f", msg->pose.position.x, msg->pose.position.y);
                                                                            goal_pose = msg->pose;
                                                                            is_goal_set = true;
                                                                        });

    ros::Subscriber map_sub = nh.subscribe<nav_msgs::OccupancyGrid>("/ow/global_map", 1,
                                                                    [&](const nav_msgs::OccupancyGrid::ConstPtr &msg)
                                                                    {
                                                                        grid_map = *msg;
                                                                        has_grid_map = true;
                                                                        std::cout << "Map h " << msg->info.height << "w =" << msg->info.width << std::endl;
                                                                    });

    std::unique_ptr<AStar> a_star_ptr = std::make_unique<AStar>(nh, is_visualization, is_eight_connected);

    AStarResult a_star_result;

    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        ros::spinOnce();
        if (!is_start_set || !is_goal_set || !has_grid_map)
        {
            if (!is_start_set)
            {
                ROS_WARN_THROTTLE(5, "Waiting for start pose topic ...");
            }
            if (!is_goal_set)
            {
                ROS_WARN_THROTTLE(5, "Waiting for goal pose topic ...");
            }
            if (!has_grid_map)
            {
                ROS_WARN_THROTTLE(5, "Waiting for grid map topic ...");
            }
            loop_rate.sleep();
            continue;
        }

        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        visualization_msgs::MarkerArray marker_array;
        int id = 0;

        a_star_ptr->SetMap(grid_map);
        if (!is_point_robot)
        {
            // 如果不是点机器人, 则需要设置长宽等参数
            a_star_ptr->SetVhicleParams(2.0, 1.0, 0.0);
        }
        ros::Time start_time = ros::Time::now();

        if (a_star_ptr->SearchPath(start_pose.position.x, start_pose.position.y,
                                   goal_pose.position.x, goal_pose.position.y,&a_star_result))
        {

            double delta_time = (ros::Time::now() - start_time).toSec();
            std::cout << "[查找路径话费时间 delta_time =  " << delta_time << " s]!" << std::endl;

            for (size_t i = 0; i < a_star_result.x.size(); i++)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = a_star_result.x[i];
                pose.pose.position.y = a_star_result.y[i];
                path.poses.push_back(std::move(pose));
                if (!is_point_robot && is_visualization)
                {
                    visualization_msgs::Marker circle_marker;
                    circle_marker.header.frame_id = "map";
                    circle_marker.header.stamp = ros::Time::now();
                    circle_marker.id = id++;
                    circle_marker.action = visualization_msgs::Marker::ADD;
                    circle_marker.lifetime = ros::Duration();
                    circle_marker.type = visualization_msgs::Marker::CYLINDER;
                    circle_marker.color.r = 1.0;
                    circle_marker.color.a = 0.5;
                    circle_marker.scale.x = std::sqrt(1.0 * 1.0 + 0.5 * 0.5);
                    circle_marker.scale.y = std::sqrt(1.0 * 1.0 + 0.5 * 0.5);
                    circle_marker.scale.z = 0.01;
                    circle_marker.pose.position.x = a_star_result.x[i];
                    circle_marker.pose.position.y = a_star_result.y[i];
                    marker_array.markers.push_back(std::move(circle_marker));
                }
            }
            path_pub.publish(path);
            if (!is_point_robot && is_visualization)
            {
                circle_pub.publish(marker_array);
            }
            is_goal_set = false;
        }
        loop_rate.sleep();
    }
    return 0;
}
