#include <queue>
#include <boost/heap/binomial_heap.hpp>
#include "pkg_nav/global_paln/a_star_search.h"

AStar::AStar(const ros::NodeHandle &nh,
             const bool visualization,
             const bool is_eight_connected) : nh_(nh),
                                            //   is_visualization_(visualization),
                                              is_eight_connected_(is_eight_connected)
{
    // visualization_pub_ = nh_.advertise<visualization_msgs::Marker>("a_star_vis", 1);
    // marker_.header.frame_id = "map";
    // marker_.id = 1;
    // marker_.action = visualization_msgs::Marker::ADD;
    // marker_.lifetime = ros::Duration();
    // marker_.type = visualization_msgs::Marker::SPHERE_LIST;
    // marker_.color.r = 1.0;
    // marker_.color.a = 1.0;
    // marker_.scale.x = 0.2;
    // marker_.scale.y = 0.2;
    // marker_.scale.z = 0.2;
}

std::vector<std::shared_ptr<Node2d>>
AStar::GenerateNextNodes(
    std::shared_ptr<Node2d> node) const
{
    std::vector<std::shared_ptr<Node2d>> next_nodes;
    const int cur_index_x = node->GetIndexX();
    const int cur_index_y = node->GetIndexY();
    const double xy_resolution = grid_map_.info.resolution;
    int index = 1;
    auto up = std::make_shared<Node2d>(cur_index_x, cur_index_y +  index);
    //    up->SetGcost(node->GetGCost() + xy_resolution);
    auto down = std::make_shared<Node2d>(cur_index_x, cur_index_y - index);
    // down->SetGcost(node->GetGCost() + xy_resolution);
    auto right = std::make_shared<Node2d>(cur_index_x + index, cur_index_y);
    //    right->SetGcost(node->GetGCost() + xy_resolution);
    auto left = std::make_shared<Node2d>(cur_index_x - index, cur_index_y);
    //    left->SetGcost(node->GetGCost() + xy_resolution);
    next_nodes.emplace_back(std::move(up));
    next_nodes.emplace_back(std::move(down));
    next_nodes.emplace_back(std::move(right));
    next_nodes.emplace_back(std::move(left));
    if (is_eight_connected_)
    { // 八连通时
        auto up_right = std::make_shared<Node2d>(cur_index_x + 1 * index, cur_index_y +  index);
        //    up_right->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
        auto up_left = std::make_shared<Node2d>(cur_index_x - 1 * index, cur_index_y + index);
        //    up_left->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
        auto down_right = std::make_shared<Node2d>(cur_index_x +  index, cur_index_y - index);
        //    down_right->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
        auto down_left = std::make_shared<Node2d>(cur_index_x - index, cur_index_y - index);
        //    down_left->SetGcost(node->GetGCost() + std::sqrt(2.0) * xy_resolution);
        next_nodes.emplace_back(std::move(up_right));
        next_nodes.emplace_back(std::move(up_left));
        next_nodes.emplace_back(std::move(down_right));
        next_nodes.emplace_back(std::move(down_left));
    }
    return next_nodes;
}

bool AStar::VerifyNode2d(std::shared_ptr<Node2d> node) const
{
    if (is_point_robot_)
    {
        return CheckPose2d(node->GetIndexX(), node->GetIndexY(), grid_map_);
    }
    else
    {
        double x, y;
        // 或许应该使用更精确的方法, 但仅是一个全局路径, 这样似乎也能接受
        const double raidus = std::hypot(0.5 * length_ + axle_ref_x_, 0.5 * width_);
        Index2Pose(node->GetIndexX(), node->GetIndexY(), grid_map_, &x, &y);
        return CheckCircleRobotPose(x, y, raidus, grid_map_);
    }
}

bool AStar::TracePath(AStarResult *astar_path)
{
    if (astar_path == nullptr)
    {
        ROS_FATAL("[AStar::TracePath]: the input pointer is nullptr");
        return false;
    }
    if (final_node_ == nullptr)
    {
        ROS_FATAL("[AStar::TracePath]: the final node is nullptr, cannot find path");
        return false;
    }
    std::vector<double> xs, ys;
    auto current_node = final_node_;
    double x, y;

    while (current_node->GetPreNode() != nullptr)
    {
        Index2Pose(
            current_node->GetIndexX(),
            current_node->GetIndexY(),
            grid_map_,
            &x, &y);
        /* debug
        std::cout << " index_x : " << current_node->GetIndexX()
                << " index_y: " << current_node->GetIndexY()
                << " x: " << x << " y : " << y << std::endl;
        */
        xs.push_back(x);
        ys.push_back(y);
        current_node = current_node->GetPreNode();
    }
    double x_start, y_start;
    Index2Pose(
        current_node->GetIndexX(),
        current_node->GetIndexY(),
        grid_map_,
        &x_start,
        &y_start);
    xs.push_back(x_start);
    ys.push_back(y_start);
    std::reverse(xs.begin(), xs.end());
    std::reverse(ys.begin(), ys.end());
    astar_path->x = std::move(xs);
    astar_path->y = std::move(ys);
    double len = 0.0;
    for (size_t i = 1; i < astar_path->x.size(); ++i)
    {
        len += std::hypot(astar_path->x[i] - astar_path->x[i - 1],
                          astar_path->y[i] - astar_path->y[i - 1]);
    }
    astar_path->cost = len;
    return true;
}

bool AStar::SearchPath(const double sx, const double sy,
                       const double ex, const double ey, AStarResult *result, int heuristic)
{

    if (result == nullptr)
    {
        ROS_FATAL("[AStar::SearchPath], the input pointer is nullptr!");
        return false;
    }
    if (!has_map_)
    {
        ROS_FATAL("[AStar::SearchPath], don't have grid map!");
        return false;
    }

    result->x.clear();
    result->y.clear();

    boost::heap::binomial_heap<std::shared_ptr<Node2d>, boost::heap::compare<cmp>> priority_queue;
    std::unordered_map<std::string, boost::heap::binomial_heap<std::shared_ptr<Node2d>, boost::heap::compare<cmp>>::handle_type> open_set_handles;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> open_set;
    std::unordered_map<std::string, std::shared_ptr<Node2d>> close_set;
    std::shared_ptr<Node2d> start_node = std::make_shared<Node2d>(sx, sy, grid_map_);
    std::shared_ptr<Node2d> end_node = std::make_shared<Node2d>(ex, ey, grid_map_);

    if (!VerifyNode2d(start_node))
    {
        ROS_FATAL("[AStar::SearchPath], the start node is not collision free");
        return false;
    }
    if (!VerifyNode2d(end_node))
    {
        ROS_FATAL("[AStar::SearchPath], the end node is not collision free");
        return false;
    }

    start_node->SetHCost(heuristic*EuclidDistance(start_node, end_node));

    final_node_ = nullptr;
    auto handle = priority_queue.emplace(start_node);
    open_set_handles.emplace(start_node->GetIndex(), handle);

    // if (is_visualization_)
    // {
    //     marker_.header.stamp = ros::Time::now();
    //     marker_.points.clear();
    // }
    while (!priority_queue.empty())
    {
        auto current_node = priority_queue.top();
        priority_queue.pop();
        open_set_handles.erase(current_node->GetIndex());
        open_set.erase(current_node->GetIndex());

        if (*(current_node) == *(end_node))
        {
            final_node_ = current_node;
            break;
        }
        close_set.emplace(current_node->GetIndex(), current_node);
        auto next_nodes = GenerateNextNodes(current_node);
        for (auto &next_node : next_nodes)
        {
            if (!VerifyNode2d(next_node))
            {
                continue;
            }
            if (close_set.find(next_node->GetIndex()) != close_set.end())
            {
                continue;
            }
            double tentative_g = current_node->GetGCost() +
                                 EdgeCost(current_node, next_node);
            if (open_set.find(next_node->GetIndex()) == open_set.end())
            {
                next_node->SetHCost(heuristic*EuclidDistance(next_node, end_node));
                next_node->SetPreNode(current_node);
                next_node->SetGcost(tentative_g);
                open_set.emplace(next_node->GetIndex(), next_node);
                auto next_handle = priority_queue.emplace(next_node);
                open_set_handles.emplace(next_node->GetIndex(), next_handle);

                // if (is_visualization_)
                // {
                //     geometry_msgs::Point point;
                //     Index2Pose(next_node->GetIndexX(), next_node->GetIndexY(),
                //                grid_map_, &point.x, &point.y);
                //     marker_.points.push_back(std::move(point));
                //     visualization_pub_.publish(marker_);
                // }
            }
            else if (open_set[next_node->GetIndex()]->GetGCost() > tentative_g)
            {
                // open_set[next_node->GetIndex()] = next_node;
                open_set[next_node->GetIndex()]->SetGcost(tentative_g);
                open_set[next_node->GetIndex()]->SetPreNode(current_node);
                priority_queue.update(open_set_handles[next_node->GetIndex()],
                                      open_set[next_node->GetIndex()]);
            }
        }
    }

    if (final_node_ == nullptr)
    {
        ROS_FATAL("[AStar::SearchPath]:, the final node is nullptr, Failed to find Path");
        return false;
    }
    this->TracePath(result);
    return true;
}