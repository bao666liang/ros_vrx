#ifndef A_STAR_SEARCH_H
#define A_STAR_SEARCH_H

#include <utility>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>
#include <map>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h> //src/pkg_nav/include/pkg_nav/global_paln/a_star_search.h

#include "a_star_utils.h"

class Node2d
{
public:
    //////////////////// constructor /////////////////////////////
    Node2d(
        const double x, const double y,
        const nav_msgs::OccupancyGrid &grid_map)
    {
        Pose2Index(x, y, std::move(grid_map), &index_x_, &index_y_);
        index_ = std::to_string(index_x_) + "_" + std::to_string(index_y_);
    }
    Node2d(int x, int y)
    {
        index_x_ = x;
        index_y_ = y;
        index_ = std::to_string(index_x_) + "_" + std::to_string(index_y_);
    }
    //////////////////// setter /////////////////////////////
    void SetHCost(const double h)
    {
        hcost_ = h;
        fcost_ = hcost_ + gcost_;
    }
    void SetFcost(const double f) { fcost_ = f; }
    void SetGcost(const double g)
    {
        gcost_ = g;
        fcost_ = gcost_ + hcost_;
    }
    void SetPreNode(std::shared_ptr<Node2d> pre_node)
    {
        pre_node_ = std::move(pre_node);
    }
    /////////////////// getter ///////////////////////
    inline double GetHCost() const { return hcost_; }
    inline double GetFCost() const { return fcost_; }
    inline double GetGCost() const { return gcost_; }
    inline int GetIndexX() const { return index_x_; }
    inline int GetIndexY() const { return index_y_; }
    inline const std::string &GetIndex() const { return index_; }
    inline std::shared_ptr<Node2d> GetPreNode() const { return pre_node_; }
    /////////////// operator ////////////////////////
    bool operator==(const Node2d &other) const
    {
        return other.GetIndex() == index_;
    }

private:
    double gcost_ = 0.0;
    double fcost_ = 0.0;
    double hcost_ = 0.0;
    int index_x_ = 0;
    int index_y_ = 0;
    std::string index_;
    std::shared_ptr<Node2d> pre_node_ = nullptr;
};

struct AStarResult
{
    std::vector<double> x;
    std::vector<double> y;
    double cost = 0.0;
};

class AStar
{
public:
    //////////////////// constructor /////////////////////////////
    AStar(const ros::NodeHandle &nh, const bool visualization, const bool is_eight_connected);
    ~AStar() = default;

    void SetVhicleParams(const double length, const double width, const double axle_ref_x)
    {
        length_ = length;
        width_ = width;
        axle_ref_x_ = axle_ref_x;
        is_point_robot_ = false;
    }

    bool SearchPath(const double sx, const double sy,
                    const double ex, const double ey, AStarResult *result, int heuristic = 1);

    void SetMap(const nav_msgs::OccupancyGrid &grid_map)
    {
        grid_map_ = std::move(grid_map);
        has_map_ = true;
        xy_resolution_ = grid_map_.info.resolution;
    }

protected:
    // 欧式距离
    double EuclidDistance(std::shared_ptr<Node2d> cur,
                          std::shared_ptr<Node2d> next) const
    {
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        const double h = std::hypot(next_x - cur_x, next_y - cur_y);
        const double tie_breaker = 1; // 暂时为1 todo
        return tie_breaker * h;
    }

    // 曼哈顿距离
    double MahDistance(std::shared_ptr<Node2d> cur,
                       std::shared_ptr<Node2d> next) const
    {
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        return std::fabs(next_x - cur_x) + std::fabs(next_y - cur_y);
    }

    // 暂时与欧式距离一致
    double EdgeCost(std::shared_ptr<Node2d> cur,
                    std::shared_ptr<Node2d> next) const
    {
        const int cur_index_x = cur->GetIndexX();
        const int cur_index_y = cur->GetIndexY();
        const int next_index_x = next->GetIndexX();
        const int next_index_y = next->GetIndexY();
        double cur_x, cur_y, next_x, next_y;
        Index2Pose(cur_index_x, cur_index_y, grid_map_, &cur_x, &cur_y);
        Index2Pose(next_index_x, next_index_y, grid_map_, &next_x, &next_y);
        return std::hypot(next_x - cur_x, next_y - cur_y);
        ;
    }

    bool VerifyNode2d(std::shared_ptr<Node2d> node) const;

    std::vector<std::shared_ptr<Node2d>>
    GenerateNextNodes(std::shared_ptr<Node2d> node) const;

    bool TracePath(AStarResult *astar_path);

    // 重载运算符，用来生成节点的比较逻辑，该函数在“boost::heap::compare<cmp>”获得使用
    struct cmp
    {
        bool operator()(const std::shared_ptr<Node2d> left, const std::shared_ptr<Node2d> right) const
        {
            return left->GetFCost() >= right->GetFCost();
        }
    };

private:
    double length_{};
    double width_{};
    double axle_ref_x_{};
    double xy_resolution_{};
    double node_radius_{};
    std::shared_ptr<Node2d> start_node_ = nullptr;
    std::shared_ptr<Node2d> goal_node_ = nullptr;
    std::shared_ptr<Node2d> final_node_ = nullptr;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> dp_map_;
    bool has_map_ = false;
    bool is_eight_connected_ = true;
    bool is_visualization_ = false;
    bool is_point_robot_ = true;
    nav_msgs::OccupancyGrid grid_map_;

    // just visualization
    ros::NodeHandle nh_;
    ros::Publisher visualization_pub_;
    visualization_msgs::Marker marker_;
};

#endif //A_STAR_SEARCH_H
