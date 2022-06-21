#include "ReferenceLine/reference_line_smooth.h"

bool ReferenceLineSmooth::Smooth(
        std::vector<PathPoint> &raw_path, 
        std::vector<PathPoint> *smooth_path) {
    if (raw_path.empty()) {
        ROS_ERROR_STREAM("the size of raw path is too small!!!");
        return false;
    } else if (raw_path.size() < 5) {
        ROS_WARN_STREAM("the size of raw path is small, don't need smooth!!!");
        *smooth_path = raw_path;
        return true;
    }

    // std::cout << "size : " << raw_path.size() << std::endl;
    std::vector<std::pair<double, double>>  path; 
    for (size_t i = 0; i < raw_path.size(); i++) {
        // std::cout << "x:" << raw_path[i].x() << ", y:" << raw_path[i].y() <<std::endl;
        path.push_back({raw_path[i].x(), raw_path[i].y()});
    }
    std::vector<double> bounds(raw_path.size(), 5.0);
    bounds.front() = 0.1;
    bounds.back() = 0.1;

    ROS_ASSERT(path.size() == bounds.size());    
    FemPosDeviationSqpOsqpInterface osqp_interface_;
    osqp_interface_.set_ref_points(path); // 设置参考点
    osqp_interface_.set_bounds_around_refs(bounds);  // 设置边界
    if (!osqp_interface_.Solve()) {
        ROS_ERROR_STREAM("OSQP Failed !");
        return false;
    }
    std::vector<std::pair<double, double>> opt_xy = osqp_interface_.opt_xy();
    for (size_t i = 0; i < opt_xy.size(); ++i) {
        PathPoint pts;
        pts.set_x(opt_xy[i].first);
        pts.set_y(opt_xy[i].second);
        smooth_path->push_back(std::move(pts));
    }
    return true;
}