#ifndef A_STAR_UTILS_H
#define A_STAR_UTILS_H

#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_datatypes.h>

namespace
{

    // 将(x, y)方向的栅格索引，转换为真实的坐标
    void Index2Pose(const int index_x, const int index_y,
                    const nav_msgs::OccupancyGrid &grid_map,
                    double *x, double *y)
    {
        const double theta = tf::getYaw(grid_map.info.origin.orientation);
        const double robot_x = index_x * grid_map.info.resolution;
        const double robot_y = index_y * grid_map.info.resolution;
        const double relative_x = std::cos(theta) * robot_x - std::sin(theta) * robot_y;
        const double relative_y = std::sin(theta) * robot_x + std::cos(theta) * robot_y;
        *x = grid_map.info.origin.position.x + relative_x;
        *y = grid_map.info.origin.position.y + relative_y;
    }

    // 获取坐标(x，y)索引，并填充进(index_x, index_y)
    void Pose2Index(const double x, const double y,
                    const nav_msgs::OccupancyGrid &grid_map,
                    int *index_x, int *index_y)
    {
        const double origin_x = grid_map.info.origin.position.x;
        const double origin_y = grid_map.info.origin.position.y;
        const double origin_theta = tf::getYaw(grid_map.info.origin.orientation);
        const double dx = x - origin_x;
        const double dy = y - origin_y;
        const double robot_x = std::cos(origin_theta) * dx + std::sin(origin_theta) * dy;
        const double robot_y = -std::sin(origin_theta) * dx + std::cos(origin_theta) * dy;
        *index_x = static_cast<int>(std::round(robot_x / grid_map.info.resolution)); // 实际的距离/栅格地图的分辨率 = 栅格个数
        *index_y = static_cast<int>(std::round(robot_y / grid_map.info.resolution));
    }

    // 判断栅格索引(index_x, index_y)是否在栅格地图内
    bool InBoudary(const int index_x, const int index_y,
                   const nav_msgs::OccupancyGrid &grid_map)
    {
        return index_x - 5 / grid_map.info.resolution >= 0 && index_x + 5 / grid_map.info.resolution < grid_map.info.width &&
               index_y - 5 / grid_map.info.resolution >= 0 && index_y + 5 / grid_map.info.resolution < grid_map.info.height;
    }

    // 根据x，y方向的栅格个数，计算所在位置处的索引
    inline int CalcIndex(const int index_x, const int index_y,
                         const nav_msgs::OccupancyGrid &grid_map)
    {
        return index_y * grid_map.info.width + index_x;
    }

    // 检查实际的坐标(x, y)是否有效（是否超出边界，或是否障碍物）
    bool CheckPose2d(const int index_x, const int index_y,
                     const nav_msgs::OccupancyGrid &grid_map)
    {
        if (!InBoudary(index_x, index_y, grid_map))
        { // 判断索引是否超出栅格地图的边界
            return false;
        }
        const int index = CalcIndex(index_x, index_y, grid_map); // 计算在一维数组中的索引
        // if (grid_map.data[index] != 0)
        // {
        //     return false;
        // }
        // int indexOffset = 1 / grid_map.info.resolution;
        // for (size_t i = 1; i < 6+1; i++)
        // {
        //    if (grid_map.data[index + i*indexOffset] != 0 || grid_map.data[index - i*indexOffset] != 0)
        //    {
        //        return false;
        //    }
        //    if (grid_map.data[i * indexOffset * grid_map.info.width + index] != 0 || grid_map.data[index - i * indexOffset * grid_map.info.width] != 0)
        //    {
        //        return false;
        //    }
        // }
        return grid_map.data[index] == 0; //TODO
    }

    // 检查以(x, y)为中心，raidus为半径的机器人所在位置是否有效
    bool CheckCircleRobotPose(const double x, const double y, const double raidus,
                              const nav_msgs::OccupancyGrid &grid_map)
    {
        double robot_x;
        double robot_y;
        int index_x, index_y;
        for (double theta = -M_PI; theta < M_PI; theta += 1.0 / 6.0 * M_PI)
        {
            for (double r = 0.0; r <= raidus; r += 1 / grid_map.info.resolution)
            {
                robot_x = x + r * std::cos(theta);
                robot_y = y + r * std::sin(theta);
                int index_x, index_y;
                Pose2Index(robot_x, robot_y, grid_map, &index_x, &index_y);
                if (!CheckPose2d(index_x, index_y, grid_map))
                {
                    return false;
                }
            }
        }
        return true;
        // int indexOffset = 1 / grid_map.info.resolution;
        // for (size_t i = 1; i < 5+1; i++)
        // {
        //     robot_x = x + i * indexOffset;
        //     robot_y = y + i * indexOffset;
        //     Pose2Index(robot_x, robot_y, grid_map, &index_x, &index_y);
        //     if (!CheckPose2d(index_x, index_y, grid_map))
        //     {
        //         return false;
        //     }
        //     robot_x = x - i * indexOffset;
        //     robot_y = y - i * indexOffset;
        //     Pose2Index(robot_x, robot_y, grid_map, &index_x, &index_y);
        //     if (!CheckPose2d(index_x, index_y, grid_map))
        //     {
        //         return false;
        //     }
        // }
        // return true;
    }

} // namespace name
#endif //A_STAR_UTILS_H