#include <cmath>
#include <time.h>
#include <iostream>
#include <string>
#include <stdio.h> 
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Path.h>
#include"PID_Controller.hpp"


std_msgs::Bool Move_flag;   
nav_msgs::Odometry g_Odometry;
nav_msgs::Path loaclPath;  
float base_link_x,base_link_y,base_link_z;
float theta; 
float PrepointAngle;



void caculateMoveCmd(); 
ros::Publisher ThrustAngleCmd_pub; 

struct POINT
{
    float  x;    
    float  y;
};
int count = 0;
float Pid_Control(float LosAngle,float theta)
{
    float  Err = (LosAngle - theta);
    static float ErrRate = 0;
    static float ErrInt = 0;
    static float ErrLastTime = 0;
    // 保证角度在[-180，180]  
    if(Err< -180)
    {
        Err = Err + 360;
    }
    if(Err > 180)
    {
        Err = Err - 360;
    }

    if(count%10 == 0)
    {       
	    ErrRate = (Err - ErrLastTime);
        ErrInt = (Err + ErrInt);
	    ErrLastTime = Err; //两次采样时间间隔为count循环十一次的时间
    }
    count++;
    //误差要取负号   
    float Angle  = -(0.03*Err+ 30*ErrRate);
    //float Angle  = -(0.05*Err + 0.000001*ErrInt );

      //  船舶 PD控制                    
           
    return Angle;
}

void simulationCmdPub(ros::Publisher pub,float cmdangle)
{
    geometry_msgs::Twist twist;

    if (Move_flag.data )
    {
        twist.linear.x = 1;
    }
    else
    {   
        twist.linear.x = 0;
    }

    twist.angular.z = cmdangle;    //角度控制量默认截取在  [ -pi，pi  ] 弧度制
    // if(twist.angular.z>0.5)
    // {
    //     twist.angular.z = 0.5;
    // }   
    // if(twist.angular.z < -0.5)
    // {
    //     twist.angular.z = -0.5;
    // }   
    // ROS_INFO("发布控制指令");
    pub.publish(twist);
}





void odomCallback(const nav_msgs::Odometry &currentOdom)
{
    g_Odometry = currentOdom;
}


/*
 将局部路径数据解析
*/
void pathCallback(const nav_msgs::Path &path)
{
    loaclPath = path;
     int pathLength = loaclPath.poses.size();
    /*
    nav_msgs/Path.msg消息格式：
    Header header
         uint32 seq
         time stamp
         string frame_id
     geometry_msgs/PoseStamped[] poses
          geometry_msgs/Pose pose 
                float64 x
                float64 y
                float64 z
                float64 w
    */

    ROS_INFO("接收到局部路径，长度为%d",pathLength);
    Move_flag.data = true;
}



/*
计算theta角
*/
float caculateTheta()
{
 
    tf::Quaternion quat;
    base_link_x=g_Odometry.pose.pose.position.x;
    base_link_y=g_Odometry.pose.pose.position.y;
    base_link_z=g_Odometry.pose.pose.position.z;

    quat.setW(g_Odometry.pose.pose.orientation.w);
    quat.setX(g_Odometry.pose.pose.orientation.x);
    quat.setY(g_Odometry.pose.pose.orientation.y);
    quat.setZ(g_Odometry.pose.pose.orientation.z);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);//四元数转rpy
    float m_theta = (yaw/3.1415926)*180; //弧度转角度
    // ROS_INFO("m_theta = %f",m_theta);
    return m_theta;//yaw
}




/*
计算当前时刻以后的摸个时刻的轨迹角度与现在角度之差
*/
float ChangeAngleCal(float x ,float y,int pathLength)         
{
    float MinLength = 100000;   //最小距离初始化
    float Length;
    int MinIndex = 0;           //最小距离的点的序号
    float xx;
    float yy;
/*计算规划轨迹上离车辆当前位置最近的点和距离
     垂线最近 即求ye    

     随着船不断靠近路径，ye会逐渐减小但detla不变，
     所以期望航向角会收敛至0跟踪上路径，船的运动轨迹为曲线
*/
    for(int i = 0;i < pathLength - 5;i++)
    {
        xx = loaclPath.poses[i].pose.position.x;
        yy = loaclPath.poses[i].pose.position.y;
        Length = pow((x - xx)*(x - xx) + (y - yy)*(y - yy),0.5);
        if(Length < MinLength)
        {
            MinIndex = i;//船与路径垂点的索引
            MinLength = Length;//ye
        }
    }
    int predictIndex = 0;//前视点的索引
    float sundistance = 0;//前视距离detla  这里用的基本los是不变的
    for(int j= MinIndex;j < pathLength - 3;j++)
    {
        float x1,x2,y1,y2;
        x1 = loaclPath.poses[j].pose.position.x;
        y1 = loaclPath.poses[j].pose.position.y;
        x2 = loaclPath.poses[j+1].pose.position.x;
        y2 = loaclPath.poses[j+1].pose.position.y;
        Length = pow((x2 - x1)*(x2 - x1) + (y2 - y1)*(y2 - y1),0.5);
        sundistance += Length;
        if(sundistance > 5)
        {
            predictIndex = j+1;
            break;
        }
    }
    POINT m_Vector;
    m_Vector.x = loaclPath.poses[predictIndex].pose.position.x - x;
    m_Vector.y = loaclPath.poses[predictIndex].pose.position.y - y;
    float angle;
    /*
    atan2()的值域是[-pi, pi]  值为弧度。也正因为atan2()需要确定目标角的象限，所以atan2的参数是以(y,x)的方式指定，
    因此atan2(y,x)与atan2(-y,-x)所给出的结果是不一样的，虽然(y/x) = ((-y)/(-x))  csdn收藏
    这里世界坐标在地图左下角      这里竖轴为y轴!!!!!!
    */
    angle = atan2(m_Vector.y,m_Vector.x);
    angle = (angle/3.1415926)*180;
    return angle;//期望航向角
}


void caculateMoveCmd()     
{
    if(Move_flag.data)//接受到路径信息后才开始计算和控制
    {
        int pathLength = loaclPath.poses.size();
        theta = caculateTheta(); 
        PrepointAngle = ChangeAngleCal(base_link_x ,base_link_y,pathLength) ;//LOS输出期望航向角
        float Angle = Pid_Control(PrepointAngle,theta); //PID输出角度控制量
        printf("角度控制量%2f",Angle);
        // geometry_msgs::Twist twist;
        // twist.angular.z = Angle;
        // twist.linear.x = 1;
        // ThrustAngleCmd_pub.publish(twist);
        simulationCmdPub(ThrustAngleCmd_pub,Angle);
    }

}

int main(int argc, char *argv[])
{  
    setlocale(LC_ALL,"");  //中文输出
    ros::init(argc, argv, "controller_node");
    Move_flag.data = false;
    ros::NodeHandle node_handle;
    ros::Subscriber odom_sub = node_handle.subscribe("odom", 10, odomCallback);  //订阅VCU反馈的车辆状态
    ros::Subscriber localPath_sub = node_handle.subscribe("/ow/local_path", 10, pathCallback);  //接收规划节点的局部路劲信息
    ThrustAngleCmd_pub = node_handle.advertise<geometry_msgs::Twist>("/cmd_vel",10, true);  //将计算好的控制量发布
    ros::Rate loop_rate(50);
    while (ros::ok())
    {
        caculateMoveCmd();
        
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

