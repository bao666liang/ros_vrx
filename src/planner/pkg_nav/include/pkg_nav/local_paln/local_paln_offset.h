#ifndef LOCAL_PALN_OFFSET_H
#define LOCAL_PALN_OFFSET_H
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
#include <vector>
#include <pkg_nav/global_paln/a_star_utils.h>
// #include <>
// #include <>

namespace LocalPlan
{
    bool IsObastclePoint(const geometry_msgs::Point point, const nav_msgs::OccupancyGrid &map)
    {
        int index_x = point.x / map.info.resolution; //CalcNoObstclePointNearby
        int index_y = point.y / map.info.resolution;
        int index = CalcIndex(index_x, index_y, map);
        return map.data[index] != 0;
    }

    bool CalcNoObstcleIndexInPath(const nav_msgs::Path path, const nav_msgs::OccupancyGrid &map, int mIndexOffsetLocalGoalPose, size_t &goalIndex)
    {
        int index_offsetTimes = 0;
        while (index_offsetTimes < 5) //TODO  计非障碍物目标点 最大迭代次数
        {
            int index_x = path.poses[goalIndex].pose.position.x / map.info.resolution; //TODO => local_map
            int index_y = path.poses[goalIndex].pose.position.y / map.info.resolution;
            int index = CalcIndex(index_x, index_y, map);
            if (map.data[index] != 0)
            {
                goalIndex += mIndexOffsetLocalGoalPose / 4; //mIndexOffsetLocalGoalPose
                if (goalIndex >= path.poses.size() - 1)
                {
                    goalIndex = path.poses.size() - 1;
                }
                index_offsetTimes++;
            }
            else
            {
                break;
            }
        }
        if (index_offsetTimes < 5)
        {
            return true; //目标点为障碍物
        }
        return false;
    }

    bool FindObastacleIndexInPath(const nav_msgs::Path &path, const nav_msgs::OccupancyGrid &map, std::vector<std::pair<size_t, size_t>> &indexObastcleInGlobalPath)
    {
        // ====================================
        int index_x = 0;
        int index_y = 0;
        int index = 0;
        bool firstPoint = false;
        bool secondPoint = false;
        size_t indexfirst, indexSecond;
        for (size_t i = 0; i < path.poses.size(); i++)
        {
            index_x = path.poses[i].pose.position.x / map.info.resolution; //TODO => local_map
            index_y = path.poses[i].pose.position.y / map.info.resolution;
            index = CalcIndex(index_x, index_y, map);
            if (!firstPoint && map.data[index] != 0)
            {
                firstPoint = true; //找到第一个点
                indexfirst = i;
            }
            if (firstPoint)
            {
                if (!secondPoint && map.data[index] == 0)
                {
                    secondPoint = true; //找到第二个点
                    indexSecond = i;
                }
                if (i == path.poses.size() - 1) //最后一个点为障碍区
                {
                    if (map.data[path.poses.size() - 1] != 0)
                    {
                        secondPoint = true; //找到第二个点
                        indexSecond = i;
                    }
                }

                if (secondPoint)
                {
                    firstPoint = false;
                    secondPoint = false;
                    indexObastcleInGlobalPath.push_back(std::make_pair(indexfirst, indexSecond));
                }
            }
        }

        if (indexObastcleInGlobalPath.empty())
        {
            return true;
        }
        return false;
    }
};

#endif