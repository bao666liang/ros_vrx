#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class Map_process
{
private:
    ros::NodeHandle m_nh;
    nav_msgs::OccupancyGrid m_map;
    ros::Subscriber m_map_sub;     //Subscriber global map
    ros::Subscriber m_map_csh_sub; //Subscriber global map
    ros::Publisher m_map_pub;      //Subscriber global map
    unsigned m_map_len;
    bool m_map_initialized;

public:
    Map_process()
    {
        m_map_initialized = false;
        m_map_pub = m_nh.advertise<nav_msgs::OccupancyGrid>("/ow/global_map", 1);
        m_map_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>("/ow/global_map_csh", 1, &Map_process::get_map_cb, this);
        m_map_csh_sub = m_nh.subscribe<nav_msgs::OccupancyGrid>("/ow/global_map", 1, &Map_process::get_map_cb, this);

        unsigned count = 1;
        ros::Rate loop_rate(1);
        while (ros::ok)
        {
            if (m_map_initialized)
            {
                break;
            }
            ros::spinOnce();
            loop_rate.sleep();
        }
        Map_process::map_process();
        while (ros::ok)
        {
            m_map.header.seq = count;
            m_map_pub.publish(m_map);
            count++;
            loop_rate.sleep();
        }

        ros::spin();
    }

    void get_map_cb(const nav_msgs::OccupancyGrid::ConstPtr &map_msg)
    {
        m_map_initialized = false;
        unsigned m_map_len = map_msg->data.size();
        if (m_map_len == map_msg->info.height * map_msg->info.width)
        {
            m_map = *map_msg;
            ROS_INFO("The map 高*宽 = %d*%d", m_map.info.height, m_map.info.width);
            unsigned show_count = 500;
            Map_process::map_show(show_count);
            m_map_initialized = true;
        }
        else
        {
            ROS_FATAL("The map 大小(%d) != 高(%d)*宽(%d)", m_map_len, m_map.info.height, m_map.info.width);
        }
    }
    bool map_process()
    {
        unsigned m_map_len = m_map.data.size();
        for (size_t i = 0; i < m_map_len; i++)
        {
            if (int(m_map.data[i]) < 100)
                m_map.data[i] = 100;
            else
                m_map.data[i] = 0;
        }
        unsigned show_count = 500;
        Map_process::map_show(show_count);
        m_map.header.frame_id = "map";
        return true;
    }

    void map_show(unsigned data_count)
    {
        unsigned count = 0;
        for (size_t i = 0; i < data_count; i++)
        {
            std::cout << int(m_map.data[i]) << " ";
            count++;
            if (count % 50 == 0)
                std::cout << "count = " << count << std::endl;
        }
    }
};

int main(int argc, char *argv[])
{
    setlocale(LC_ALL, "");
    //执行 ros 节点初始化
    ros::init(argc, argv, "ow_process_node");
    ROS_INFO("The program of ow_process_node is running ...");
    //创建 ros 节点句柄(非必须)
    ros::NodeHandle nh;
    Map_process map_process;
    return 0;
}