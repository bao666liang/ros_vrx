#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include "pkg_nav/osqp_problem.h"
#include "pkg_nav/smoothosqpproblem.h"

using namespace std;

int main(int argc, char **argv)
{
    //防止中文乱码
    setlocale(LC_CTYPE, "zh_CN.utf8");
    // ROS节点初始化
    ros::init(argc, argv, "reference_line_smooth");

    ros::NodeHandle nh;
    ros::Publisher raw_path_pub = nh.advertise<nav_msgs::Path>("raw_path", 1);
    ros::Publisher smooth_path_pub = nh.advertise<nav_msgs::Path>("smooth_path", 1);

    std::cout << "OSQP RUNNING..." << std::endl;

    std::vector<std::pair<double, double>> raw_point2d;
    std::vector<double> bounds;
    std::vector<double> opt_x;
    std::vector<double> opt_y;

    for (int i = 0; i < 25; ++i)
    {
        raw_point2d.push_back(std::make_pair(i * 1.0, 2 * i));
    }

    for (int i = 0; i < 25; ++i)
    {
        raw_point2d.push_back(std::make_pair(25 + i, 50));
    }

    for (int i = 0; i < 25; ++i)
    {
        raw_point2d.push_back(std::make_pair(50 + i, 50 + i));
    }

    // for (int i = 0 ;i < 50; ++i) {
    //     raw_point2d.push_back(std::make_pair(i*1.0, 0.0));
    // }

    bounds.push_back(0.1);

    for (int i = 0; i < 73; ++i)
    {
        bounds.push_back(10.0);
    }

    bounds.push_back(0.1);

    nav_msgs::Path raw_path;
    raw_path.header.frame_id = "map";
    raw_path.header.stamp = ros::Time::now();
    for (size_t i = 0; i < raw_point2d.size(); ++i)
    {
        geometry_msgs::PoseStamped point;
        point.pose.position.x = raw_point2d[i].first;
        point.pose.position.y = raw_point2d[i].second;
        raw_path.poses.push_back(point);
    }

    //    //Now we are using default variables in osqp solver
    //    double weight_fem_pos_deviation = 1e10;
    //    double weight_path_length = 1.0;
    //    double weight_ref_deviation = 1.0;
    //    double weight_curvature_constraint_slack_var = 10.0;
    //    double curvature_constraint = 0.08;
    //    int sqp_sub_max_iter = 20;
    //    int sqp_pen_max_iter = 10;
    //    double sqp_ctol = 0.08;
    //    int max_iter = 500;
    //    double time_limit = 0.0;
    //    bool verbose = false;
    //    bool scaled_termination = true;
    //    bool warm_start = true;

    FemPosDeviationSqpOsqpInterface solver;
    solver.set_ref_points(raw_point2d);
    std::vector<std::pair<double, double>> mUpperBounds_around_refs;
    std::vector<std::pair<double, double>> mLowerBounds_around_refs;
    solver.set_bounds_around_refs(mUpperBounds_around_refs, mLowerBounds_around_refs);

    if (!solver.Solve())
    {
        std::cout << "failture" << std::endl;
    }

    std::vector<std::pair<double, double>> opt_xy = solver.opt_xy();

    // TODO(Jinyun): unify output data container
    opt_x.resize(opt_xy.size());
    opt_y.resize(opt_xy.size());

    nav_msgs::Path smooth_path;
    smooth_path.header.frame_id = "map";
    smooth_path.header.stamp = ros::Time::now();

    for (size_t i = 0; i < opt_xy.size(); ++i)
    {
        opt_x[i] = opt_xy[i].first;
        opt_y[i] = opt_xy[i].second;
        geometry_msgs::PoseStamped point;
        point.pose.position.x = opt_xy[i].first;
        point.pose.position.y = opt_xy[i].second;
        smooth_path.poses.push_back(point);
    }

    std::cout << "xxx::" << std::endl;
    for (int i = 0; i < opt_x.size(); ++i)
    {
        std::cout << opt_x[i] << std::endl;
    }

    std::cout << "yyy::" << std::endl;
    for (int i = 0; i < opt_y.size(); ++i)
    {
        std::cout << opt_y[i] << std::endl;
    }

    //    if(prob->Optimize(1000)){
    //      std::cout<<"Optimize successful!!"<<std::endl;

    //      for (int i = 0; i < prob->x_.size(); ++i)
    //      {
    //          mycout<<"x: "<<prob->x_.at(i)<<" dx: "<<prob->dx_.at(i)<<" ddx: "<<prob->ddx_.at(i)<<" left_edge: "<<bound_info.boundary_.at(i).second<<" right_edge: "<<bound_info.boundary_.at(i).first<<std::endl;
    //      }

    //    }else{
    //      std::cout<<"Optimize failed!!"<<std::endl;
    //    }
    std::cout << "OSQP END!!!" << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok())
    {
        raw_path_pub.publish(raw_path);
        smooth_path_pub.publish(smooth_path);

        //sleep and spin
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
