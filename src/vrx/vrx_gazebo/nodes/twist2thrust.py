#!/usr/bin/env python
# license removed for brevity

import sys
import rospy
from geometry_msgs.msg import Twist
from rospy.core import rospyinfo
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
gazebo_speed=0.0
SetLevel=0
# from speed2cmdvel import cmd_to_speed
class Node():
    def __init__(self,linear_scaling,angular_scaling,keyboard=False):
        self.linear_scaling = linear_scaling
        self.angular_scaling = angular_scaling
        self.left_pub = None
        self.right_pub = None
        self.left_msg =None
        self.right_msg =None
        self.keyboard = keyboard
        self.Kp=1.8
        self.Ki=0.01
        self.Kd=5.55
        self.SetLevel = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.windup_guard = 100.0
        self.output = 0.0
        
    def clear(self):
        self.SetLevel = 0.0
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0
        self.windup_guard = 2.2
        self.output = 0.0

    def update(self,feedback_value):
        global SetLevel
        error=SetLevel-(feedback_value)
        self.PTerm = self.Kp * error
        self.ITerm += error
        if (self.ITerm < -self.windup_guard):
            self.ITerm = -self.windup_guard
        elif (self.ITerm > self.windup_guard):
            self.ITerm = self.windup_guard
        delta_error = error-self.last_error
        self.DTerm = delta_error
        self.last_error = error
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)


    # def callback_gazebo_speed(self,data):
    #      index=data.name.index("wamv::left_engine_link")
    #      global gazebo_speed
    #      gazebo_speed=-1*data.twist[index].linear.x
             
    def callback(self,data):

        self.left_msg.data = data.linear.x
        self.right_msg.data = data.linear.x
        self.left_msg_angle = data.angular.z
        self.right_msg_angle = data.angular.z
        #global SetLevel
        #SetLevel=data.linear.x       

        #self.right_msg.data=self.output
        #self.left_msg.data=self.output
        
        node.left_pub.publish(node.left_msg)
        node.right_pub.publish(node.right_msg)
        node.left_pub_angle.publish(node.left_msg_angle)
        node.right_pub_angle.publish(node.right_msg_angle)  

        node.wamv1left_pub.publish(1.0)
        node.wamv1right_pub.publish(1.0)
        node.wamv1left_pub_angle.publish(0.1)
        node.wamv1right_pub_angle.publish(0.1)         


if __name__ == '__main__':
    rospy.init_node('twist2drive', anonymous=True)
    # ROS Parameters
    # Scaling from Twist.linear.x to (left+right)
    linear_scaling = rospy.get_param('~linear_scaling',1)
    # Scaling from Twist.angular.z to (right-left)
    angular_scaling = rospy.get_param('~angular_scaling',1)

    # rospy.loginfo("Linear scaling=%f, Angular scaling=%f"%(linear_scaling,angular_scaling))


    key = '--keyboard' in sys.argv
    node=Node(linear_scaling,angular_scaling,keyboard=key)

    # Publisher
    node.left_pub = rospy.Publisher("/wamv/thrusters/left_thrust_cmd",Float32,queue_size=10)
    node.right_pub = rospy.Publisher("/wamv/thrusters/right_thrust_cmd",Float32,queue_size=10)
    node.left_pub_angle = rospy.Publisher('/wamv/thrusters/left_thrust_angle', Float32, queue_size=1)
    node.right_pub_angle = rospy.Publisher('/wamv/thrusters/right_thrust_angle', Float32, queue_size=1)
    node.wamv1left_pub = rospy.Publisher("/wamv1/thrusters/left_thrust_cmd",Float32,queue_size=10)
    node.wamv1right_pub = rospy.Publisher("/wamv1/thrusters/right_thrust_cmd",Float32,queue_size=10)
    node.wamv1left_pub_angle = rospy.Publisher('/wamv1/thrusters/left_thrust_angle', Float32, queue_size=1)
    node.wamv1right_pub_angle = rospy.Publisher('/wamv1/thrusters/right_thrust_angle', Float32, queue_size=1)
    node.left_msg = Float32()
    node.left_msg = Float32()
    node.right_msg = Float32()
    node.left_msg_angle = Float32()
    node.right_msg_angle = Float32()
    # Subscriber
    rospy.Subscriber("cmd_vel",Twist,node.callback)
    # rospy.Subscriber("gazebo_speed/speedhead",Float32,node.callback_gazebo_speed)
    # rospy.Subscriber("wamv/robot_localization/odometry/filtered", Odometry, node.callback_gazebo_speed)
    #rospy.Subscriber("gazebo/link_states", ModelStates, node.callback_gazebo_speed)
    try:
        rate = rospy.Rate(100) # 10hz
        while not rospy.is_shutdown():
            node.update(gazebo_speed)   
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
