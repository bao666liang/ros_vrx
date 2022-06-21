/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "pkg_nav/global_paln/astar_search.h"

AstarSearch::AstarSearch()
{
    ros::NodeHandle private_nh_("~");

    // base configs
    private_nh_.param<bool>("use_back", use_back_, true);
    private_nh_.param<bool>("use_potential_heuristic", use_potential_heuristic_, true);
    private_nh_.param<bool>("use_wavefront_heuristic", use_wavefront_heuristic_, false);
    private_nh_.param<double>("time_limit", time_limit_, 20000.0); //路径规划时间限制ms

    // robot configs
    private_nh_.param<double>("robot_length", robot_length_, 5);
    private_nh_.param<double>("robot_width", robot_width_, 3);
    private_nh_.param<double>("robot_base2back", robot_base2back_, 1.0);
    private_nh_.param<double>("minimum_turning_radius", minimum_turning_radius_, 6.0);

    // search configs
    private_nh_.param<int>("theta_size", theta_size_, 48);
    private_nh_.param<double>("angle_goal_range", angle_goal_range_, 6.0);
    private_nh_.param<double>("curve_weight", curve_weight_, 1.2);
    private_nh_.param<double>("reverse_weight", reverse_weight_, 2.00);
    private_nh_.param<double>("lateral_goal_range", lateral_goal_range_, 0.5);
    private_nh_.param<double>("longitudinal_goal_range", longitudinal_goal_range_, 2.0);

    // costmap configs
    private_nh_.param<int>("obstacle_threshold", obstacle_threshold_, 100);
    private_nh_.param<double>("potential_weight", potential_weight_, 10.0);
    private_nh_.param<double>("distance_heuristic_weight", distance_heuristic_weight_, 1.0);

    std::cout << "robot_length_ = " << robot_length_ << std::endl;

    createStateUpdateTable();
}

AstarSearch::~AstarSearch()
{
}

// state update table for hybrid astar // 创建状态更新表
void AstarSearch::createStateUpdateTable()
{
    // Vehicle moving for each angle // 以车辆当前所处的转角作为状态更新的起点
    state_update_table_.resize(theta_size_);
    double dtheta = 2.0 * M_PI / theta_size_;

    // Minimum moving distance with one state update // 每次状态更新时的最小移动距离
    //     arc  = r                       * theta
    double step = minimum_turning_radius_ * dtheta;

    for (int i = 0; i < theta_size_; i++)
    {
        double theta = dtheta * i;

        // Calculate right and left circle
        // Robot moves along these circles  // 计算无人车移动时所循的左/右圆轨迹的圆心
        double right_circle_center_x = minimum_turning_radius_ * std::sin(theta);
        double right_circle_center_y = minimum_turning_radius_ * -std::cos(theta);
        double left_circle_center_x = -right_circle_center_x;
        double left_circle_center_y = -right_circle_center_y;

        // Calculate x and y shift to next state
        //  // 计算当前当前状态到下一状态x y坐标的变化
        NodeUpdate nu;

        // forward
        nu.shift_x = step * std::cos(theta);
        nu.shift_y = step * std::sin(theta);
        nu.rotation = 0.0;
        nu.index_theta = 0;
        nu.step = step;
        nu.curve = false;
        nu.back = false;
        state_update_table_[i].emplace_back(nu); //它在末尾插入一个新元素。

        // forward right
        nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta - dtheta);
        nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta - dtheta);
        nu.rotation = -dtheta;
        nu.index_theta = -1;
        nu.step = step;
        nu.curve = true;
        nu.back = false;
        state_update_table_[i].emplace_back(nu);

        // forward left
        nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-M_PI_2 + theta + dtheta);
        nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-M_PI_2 + theta + dtheta);
        nu.rotation = dtheta;
        nu.index_theta = 1;
        nu.step = step;
        nu.curve = true;
        nu.back = false;
        state_update_table_[i].emplace_back(nu);

        if (use_back_)
        {
            // backward
            nu.shift_x = step * std::cos(theta) * -1.0;
            nu.shift_y = step * std::sin(theta) * -1.0;
            nu.rotation = 0;
            nu.index_theta = 0;
            nu.step = step;
            nu.curve = false;
            nu.back = true;
            state_update_table_[i].emplace_back(nu);

            // backward right
            nu.shift_x = right_circle_center_x + minimum_turning_radius_ * std::cos(M_PI_2 + theta + dtheta);
            nu.shift_y = right_circle_center_y + minimum_turning_radius_ * std::sin(M_PI_2 + theta + dtheta);
            nu.rotation = dtheta;
            nu.index_theta = 1;
            nu.step = step;
            nu.curve = true;
            nu.back = true;
            state_update_table_[i].emplace_back(nu);

            // backward left
            nu.shift_x = left_circle_center_x + minimum_turning_radius_ * std::cos(-1.0 * M_PI_2 + theta - dtheta);
            nu.shift_y = left_circle_center_y + minimum_turning_radius_ * std::sin(-1.0 * M_PI_2 + theta - dtheta);
            nu.rotation = dtheta * -1.0;
            nu.index_theta = -1;
            nu.step = step;
            nu.curve = true;
            nu.back = true;
            state_update_table_[i].emplace_back(nu);
        }
    }
}

void AstarSearch::initialize(const nav_msgs::OccupancyGrid &costmap)
{
    costmap_ = costmap;

    int height = costmap_.info.height;
    int width = costmap_.info.width;

    // size initialization  resize() 它修改向量的大小。 size()	返回Vector元素数量的大小。
    nodes_.resize(height);
    for (int i = 0; i < height; i++)
    {
        nodes_[i].resize(width);
    }
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            nodes_[i][j].resize(theta_size_);
        }
    }

    // cost initialization
    for (int i = 0; i < height; i++)
    {
        for (int j = 0; j < width; j++)
        {
            // Index of subscribing OccupancyGrid message
            int og_index = i * width + j;
            int cost = costmap_.data[og_index];

            // hc is set to be 0 when reset()
            if (cost == 0)
            {
                continue;
            }

            // obstacle or unknown area
            if (cost < 0 || obstacle_threshold_ <= cost)
            {
                nodes_[i][j][0].status = STATUS::OBS;
            }

            // the cost more than threshold is regarded almost same as an obstacle
            // because of its very high cost
            //isuse potential cost function
            if (use_potential_heuristic_)
            {
                nodes_[i][j][0].hc = cost * potential_weight_;
                ////潜在启发代价hc (原始代价*权重值potential_weight_)
            }
        }
    }
    std::cout << "AstarSearch::initialize 完成 ！" << std::endl;
}

bool AstarSearch::makePlan(const geometry_msgs::Pose &start_pose, const geometry_msgs::Pose &goal_pose)
{
    if (!setStartNode(start_pose))
    {
        ROS_DEBUG("Invalid start pose");
        return false;
    }

    if (!setGoalNode(goal_pose))
    {
        ROS_DEBUG("Invalid goal pose");
        return false;
    }

    return search();
}

bool AstarSearch::setStartNode(const geometry_msgs::Pose &start_pose)
{
    // Get index of start pose
    int index_x, index_y, index_theta;
    start_pose_local_.pose = start_pose;
    poseToIndex(start_pose_local_.pose, &index_x, &index_y, &index_theta);
    SimpleNode start_sn(index_x, index_y, index_theta, 0, 0);

    // Check if start is valid
    //// isOutOfRange函数检查index_x和index_y是否位于代价地图坐标区间内
    //// detectCollision函数检查无人车所处start_sn是否位于代价地图中障碍物所在位置(将无人车视为矩形)
    if (isOutOfRange(index_x, index_y) || detectCollision(start_sn))
    {
        return false;
    }

    // Set start node
    AstarNode &start_node = nodes_[index_y][index_x][index_theta];
    start_node.x = start_pose_local_.pose.position.x;
    start_node.y = start_pose_local_.pose.position.y;
    start_node.theta = 2.0 * M_PI / theta_size_ * index_theta;
    // start_node.hc = nodes_[index_y][index_x][0].hc; //博主所加代码 cyb
    start_node.gc = 0;
    start_node.move_distance = 0;
    start_node.back = false;
    start_node.status = STATUS::OPEN;
    start_node.parent = NULL;

    // set euclidean distance heuristic cost
    if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
    {
        start_node.hc = calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                     goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                        distance_heuristic_weight_;
    }
    else if (use_potential_heuristic_)
    {
        start_node.gc += start_node.hc; //TODO cyb
        start_node.hc += calcDistance(start_pose_local_.pose.position.x, start_pose_local_.pose.position.y,
                                      goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) +
                         distance_heuristic_weight_;
        //// 此处源代码中的“+distance_heuristic_weight_”应该错了，cyb
        // 应该是“*distance_heuristic_weight_”
    }

    // Push start node to openlist
    start_sn.cost = start_node.gc + start_node.hc;
    openlist_.push(start_sn);

    return true;
}

bool AstarSearch::setGoalNode(const geometry_msgs::Pose &goal_pose)
{
    goal_pose_local_.pose = goal_pose;
    goal_yaw_ = modifyTheta(tf::getYaw(goal_pose_local_.pose.orientation));

    // Get index of goal pose
    int index_x, index_y, index_theta;
    poseToIndex(goal_pose_local_.pose, &index_x, &index_y, &index_theta);
    SimpleNode goal_sn(index_x, index_y, index_theta, 0, 0);

    // Check if goal is valid
    if (isOutOfRange(index_x, index_y) || detectCollision(goal_sn))
    {
        return false;
    }

    // Calculate wavefront heuristic cost
    if (use_wavefront_heuristic_)
    {
        // auto start = std::chrono::system_clock::now();
        bool wavefront_result = calcWaveFrontHeuristic(goal_sn);
        // auto end = std::chrono::system_clock::now();
        // auto usec = std::chrono::duration_cast<std::chrono::microseconds>(end - start).count();
        // std::cout << "wavefront : " << usec / 1000.0 << "[msec]" << std::endl;

        if (!wavefront_result)
        {
            ROS_DEBUG("Reachable is false...");
            return false;
        }
    }

    return true;
}

void AstarSearch::poseToIndex(const geometry_msgs::Pose &pose, int *index_x, int *index_y, int *index_theta)
{ //函数根据传入函数的pose以及原二维代价地图的信息，确定pose对应位置在三维代价地图中的位置。
    tf::Transform orig_tf;
    //// origin是代价栅格地图中的原点在全局地图中的位姿
    tf::poseMsgToTF(costmap_.info.origin, orig_tf);
    // transformPose是astar_util.h中的函数，astar_util.h与astar_search.cpp在同一个ROS包内
    // transformPose用于获取pose相较于代价地图中原点的相对位置
    geometry_msgs::Pose pose2d = transformPose(pose, orig_tf.inverse());
    // 确定pose在代价栅格地图中的坐标
    // resolution是栅格地图中每一格的长度
    *index_x = pose2d.position.x / costmap_.info.resolution;
    *index_y = pose2d.position.y / costmap_.info.resolution;

    tf::Quaternion quat;
    tf::quaternionMsgToTF(pose2d.orientation, quat);
    double yaw = tf::getYaw(quat);
    if (yaw < 0)
        yaw += 2.0 * M_PI;

    // Descretize angle
    // theta_size_就是前面提到的三维代价地图中的第三维的尺寸
    // 车辆位于原二维代价地图中每一格时转角不同代价也不同
    // 转角被离散为theta_size_个，间隔是one_angle_range
    static double one_angle_range = 2.0 * M_PI / theta_size_;
    *index_theta = yaw / one_angle_range;
    *index_theta %= theta_size_;
}

void AstarSearch::pointToIndex(const geometry_msgs::Point &point, int *index_x, int *index_y)
{
    geometry_msgs::Pose pose;
    pose.position = point;
    int index_theta;
    poseToIndex(pose, index_x, index_y, &index_theta);
}

bool AstarSearch::isOutOfRange(int index_x, int index_y)
{
    if (index_x < 0 || index_x >= static_cast<int>(costmap_.info.width) || index_y < 0 ||
        index_y >= static_cast<int>(costmap_.info.height))
    {
        return true;
    }
    return false;
}

bool AstarSearch::search()
{
    ros::WallTime begin = ros::WallTime::now();

    // Start A* search
    // If the openlist is empty, search failed
    // 开始基于A*算法的搜索过程
    // 首先openlist_应当非空
    // 开始搜索时，openlist_内仅有起点
    while (!openlist_.empty())
    {
        // Check time and terminate if the search reaches the time limit
        //  检查搜索时长是否超出限时
        ros::WallTime now = ros::WallTime::now();
        double msec = (now - begin).toSec() * 1000.0;
        if (msec > time_limit_)
        {
            ROS_DEBUG("Exceed time limit of %lf [ms]", time_limit_);
            return false;
        }

        // Pop minimum cost node from openlist
        // top_sn是openlist_中cost最小的SimpleNode，验证见后文
        SimpleNode top_sn = openlist_.top();
        //  // pop函数将这个cost最小的SimpleNode从openlist_中移除
        openlist_.pop(); //它将优先级最高的元素从队列中删除。

        // Expand nodes from this node
        // 每次循环都以openlist_中代价最小的节点开始探索其邻居节点
        // 因此将其设置为当前节点current_an
        AstarNode *current_an = &nodes_[top_sn.index_y][top_sn.index_x][top_sn.index_theta];
        current_an->status = STATUS::CLOSED;

        // Goal check  // 检查当前节点是否是目的节点
        if (isGoal(current_an->x, current_an->y, current_an->theta))
        {
            ROS_DEBUG("Search time: %lf [msec]", (now - begin).toSec() * 1000.0);
            // 如果是目的节点，setPath函数就从当前节点开始根据节点的parent属性往前追溯，直到起点，形成一条从起点到终点的路径
            // 路径赋值给path_，Autoware中的astar_avoid节点获取path_后将其合并进原路径，形成避障路径
            setPath(top_sn);
            return true;
        }

        // Expand nodes
        // 如果当前节点不是目的节点，以此节点为中心，向邻居节点拓展openlist_
        // state_update_table_在函数createStateUpdateTable中被初始化，对其分析详见后文
        for (const auto &state : state_update_table_[top_sn.index_theta])
        {
            // Next state // 下一状态
            double next_x = current_an->x + state.shift_x;
            double next_y = current_an->y + state.shift_y;
            double next_theta = modifyTheta(current_an->theta + state.rotation);
            double move_cost = state.step;
            double move_distance = current_an->move_distance + state.step;

            // Increase reverse cost
            if (state.back != current_an->back)
                move_cost *= reverse_weight_;

            // Calculate index of the next state   // 计算下一状态在三维代价地图nodes_中的下标
            SimpleNode next_sn;
            geometry_msgs::Point next_pos;
            next_pos.x = next_x;
            next_pos.y = next_y;
            pointToIndex(next_pos, &next_sn.index_x, &next_sn.index_y);
            next_sn.index_theta = top_sn.index_theta + state.index_theta;

            // Avoid invalid index
            next_sn.index_theta = (next_sn.index_theta + theta_size_) % theta_size_;

            // Check if the index is valid  // 避免下标越界及判断是否可行
            if (isOutOfRange(next_sn.index_x, next_sn.index_y) || detectCollision(next_sn))
            {
                continue;
            }
            // 从三维代价地图nodes_中调出对应AstarNode
            AstarNode *next_an = &nodes_[next_sn.index_y][next_sn.index_x][next_sn.index_theta];
            double next_gc = current_an->gc + move_cost;
            double next_hc = nodes_[next_sn.index_y][next_sn.index_x][0].hc; // wavefront or distance transform heuristic

            // increase the cost with euclidean distance
            if (use_potential_heuristic_)
            {
                next_gc += nodes_[next_sn.index_y][next_sn.index_x][0].hc;
                next_hc += calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                           distance_heuristic_weight_;
            }

            // increase the cost with euclidean distance
            if (!use_wavefront_heuristic_ && !use_potential_heuristic_)
            {
                next_hc = calcDistance(next_x, next_y, goal_pose_local_.pose.position.x, goal_pose_local_.pose.position.y) *
                          distance_heuristic_weight_;
            }

            // NONE  // 将节点加入openlist_
            if (next_an->status == STATUS::NONE)
            {
                next_an->status = STATUS::OPEN;
                next_an->x = next_x;
                next_an->y = next_y;
                next_an->theta = next_theta;
                next_an->gc = next_gc;
                next_an->hc = next_hc;
                next_an->move_distance = move_distance;
                next_an->back = state.back;
                next_an->parent = current_an;
                next_sn.cost = next_an->gc + next_an->hc;
                openlist_.push(next_sn);
                continue;
            }

            // OPEN or CLOSED
            if (next_an->status == STATUS::OPEN || next_an->status == STATUS::CLOSED)
            {
                if (next_gc < next_an->gc)
                {
                    next_an->status = STATUS::OPEN;
                    next_an->x = next_x;
                    next_an->y = next_y;
                    next_an->theta = next_theta;
                    next_an->gc = next_gc;
                    next_an->hc = next_hc; // already calculated ?
                    next_an->move_distance = move_distance;
                    next_an->back = state.back;
                    next_an->parent = current_an;
                    next_sn.cost = next_an->gc + next_an->hc;
                    openlist_.push(next_sn);
                    continue;
                }
            }
        } // state update
    }

    // Failed to find path
    ROS_DEBUG("Open list is empty...");
    return false;
}

void AstarSearch::setPath(const SimpleNode &goal)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = costmap_.header.frame_id;
    path_.header = header;

    // From the goal node to the start node
    AstarNode *node = &nodes_[goal.index_y][goal.index_x][goal.index_theta];

    while (node != NULL)
    {
        // Set tf pose
        tf::Vector3 origin(node->x, node->y, 0);
        tf::Pose tf_pose;
        tf_pose.setOrigin(origin);
        tf_pose.setRotation(tf::createQuaternionFromYaw(node->theta));

        // Set path as ros message
        geometry_msgs::PoseStamped ros_pose;
        tf::poseTFToMsg(tf_pose, ros_pose.pose);
        ros_pose.header = header;
        path_.poses.push_back(ros_pose);

        // To the next node
        node = node->parent;
    }

    // Reverse the vector to be start to goal order
    std::reverse(path_.poses.begin(), path_.poses.end());
}

// Check if the next state is the goal
// Check lateral offset, longitudinal offset and angle
bool AstarSearch::isGoal(double x, double y, double theta)
{
    // To reduce computation time, we use square value for distance
    static const double lateral_goal_range =
        lateral_goal_range_ / 2.0; // [meter], divide by 2 means we check left and right
    static const double longitudinal_goal_range =
        longitudinal_goal_range_ / 2.0;                                        // [meter], check only behind of the goal
    static const double goal_angle = M_PI * (angle_goal_range_ / 2.0) / 180.0; // degrees -> radian

    // Calculate the node coordinate seen from the goal point
    tf::Point p(x, y, 0);
    geometry_msgs::Point relative_node_point = calcRelativeCoordinate(goal_pose_local_.pose, p);

    // Check Pose of goal
    if (relative_node_point.x < 0 && // shoud be behind of goal
        std::fabs(relative_node_point.x) < longitudinal_goal_range &&
        std::fabs(relative_node_point.y) < lateral_goal_range)
    {
        // Check the orientation of goal
        if (calcDiffOfRadian(goal_yaw_, theta) < goal_angle)
        {
            return true;
        }
    }

    return false;
}

bool AstarSearch::isObs(int index_x, int index_y)
{
    if (nodes_[index_y][index_x][0].status == STATUS::OBS)
    {
        return true;
    }

    return false;
}

bool AstarSearch::detectCollision(const SimpleNode &sn)
{
    // Define the robot as rectangle
    static double left = -1.0 * robot_base2back_;
    static double right = robot_length_ - robot_base2back_;
    static double top = robot_width_ / 2.0;
    static double bottom = -1.0 * robot_width_ / 2.0;
    static double resolution = costmap_.info.resolution;

    // Coordinate of base_link in OccupancyGrid frame
    static double one_angle_range = 2.0 * M_PI / theta_size_;
    double base_x = sn.index_x * resolution;
    double base_y = sn.index_y * resolution;
    double base_theta = sn.index_theta * one_angle_range;

    // Calculate cos and sin in advance
    double cos_theta = std::cos(base_theta);
    double sin_theta = std::sin(base_theta);

    // Convert each point to index and check if the node is Obstacle
    for (double x = left; x < right; x += resolution)
    {
        for (double y = top; y > bottom; y -= resolution)
        {
            // 2D point rotation
            int index_x = (x * cos_theta - y * sin_theta + base_x) / resolution;
            int index_y = (x * sin_theta + y * cos_theta + base_y) / resolution;

            if (isOutOfRange(index_x, index_y))
            {
                return true;
            }
            else if (nodes_[index_y][index_x][0].status == STATUS::OBS)
            {
                return true;
            }
        }
    }

    return false;
}

bool AstarSearch::calcWaveFrontHeuristic(const SimpleNode &sn)
{
    // Set start point for wavefront search
    // This is goal for Astar search
    nodes_[sn.index_y][sn.index_x][0].hc = 0;
    WaveFrontNode wf_node(sn.index_x, sn.index_y, 1e-10);
    std::queue<WaveFrontNode> qu;
    qu.push(wf_node);

    // State update table for wavefront search
    // Nodes are expanded for each neighborhood cells (moore neighborhood)
    double resolution = costmap_.info.resolution;
    static std::vector<WaveFrontNode> updates = {
        getWaveFrontNode(0, 1, resolution),
        getWaveFrontNode(-1, 0, resolution),
        getWaveFrontNode(1, 0, resolution),
        getWaveFrontNode(0, -1, resolution),
        getWaveFrontNode(-1, 1, std::hypot(resolution, resolution)),
        getWaveFrontNode(1, 1, std::hypot(resolution, resolution)),
        getWaveFrontNode(-1, -1, std::hypot(resolution, resolution)),
        getWaveFrontNode(1, -1, std::hypot(resolution, resolution)),
    };

    // Get start index
    int start_index_x;
    int start_index_y;
    int start_index_theta;
    poseToIndex(start_pose_local_.pose, &start_index_x, &start_index_y, &start_index_theta);

    // Whether the robot can reach goal
    bool reachable = false;

    // Start wavefront search
    while (!qu.empty())
    {
        WaveFrontNode ref = qu.front();
        qu.pop();

        WaveFrontNode next;
        for (const auto &u : updates)
        {
            next.index_x = ref.index_x + u.index_x;
            next.index_y = ref.index_y + u.index_y;

            // out of range OR already visited OR obstacle node
            if (isOutOfRange(next.index_x, next.index_y) || nodes_[next.index_y][next.index_x][0].hc > 0 ||
                nodes_[next.index_y][next.index_x][0].status == STATUS::OBS)
            {
                continue;
            }

            // Take the size of robot into account
            if (detectCollisionWaveFront(next))
            {
                continue;
            }

            // Check if we can reach from start to goal
            if (next.index_x == start_index_x && next.index_y == start_index_y)
            {
                reachable = true;
            }

            // Set wavefront heuristic cost
            next.hc = ref.hc + u.hc;
            nodes_[next.index_y][next.index_x][0].hc = next.hc;

            qu.push(next);
        }
    }

    // End of search
    return reachable;
}

// Simple collidion detection for wavefront search
bool AstarSearch::detectCollisionWaveFront(const WaveFrontNode &ref)
{
    // Define the robot as square
    static double half = robot_width_ / 2;
    double robot_x = ref.index_x * costmap_.info.resolution;
    double robot_y = ref.index_y * costmap_.info.resolution;

    for (double y = half; y > -1.0 * half; y -= costmap_.info.resolution)
    {
        for (double x = -1.0 * half; x < half; x += costmap_.info.resolution)
        {
            int index_x = (robot_x + x) / costmap_.info.resolution;
            int index_y = (robot_y + y) / costmap_.info.resolution;

            if (isOutOfRange(index_x, index_y))
            {
                return true;
            }

            if (nodes_[index_y][index_x][0].status == STATUS::OBS)
            {
                return true;
            }
        }
    }

    return false;
}

void AstarSearch::reset()
{
    path_.poses.clear();

    // Clear queue
    std::priority_queue<SimpleNode, std::vector<SimpleNode>, std::greater<SimpleNode>> empty;
    std::swap(openlist_, empty);

    // ros::WallTime begin = ros::WallTime::now();

    // Reset node info here ...?
    for (size_t i = 0; i < costmap_.info.height; i++)
    {
        for (size_t j = 0; j < costmap_.info.width; j++)
        {
            for (int k = 0; k < theta_size_; k++)
            {
                // other values will be updated during the search
                nodes_[i][j][k].status = STATUS::NONE;
                nodes_[i][j][k].hc = 0;
            }
        }
    }

    // ros::WallTime end = ros::WallTime::now();

    // ROS_INFO("Reset time: %lf [ms]", (end - begin).toSec() * 1000);
}
