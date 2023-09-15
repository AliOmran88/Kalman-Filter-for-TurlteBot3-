#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 
import numpy as np 
import tf

def odometry_received(msg):

    explicit_quat = [msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w]
    print("Quaternion {}".format(explicit_quat))

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(explicit_quat)
    ydot = msg.twist.twist.angular.z
    yaw = np.random.normal(yaw, 0.25, 1)[0]
    ydot = np.random.normal(ydot, 0.3, 1)[0]
    yaw_robot = yaw 
    yaw_dot = ydot

    noisy_state = []
    noisy_state.append(yaw_robot)
    noisy_state.append(yaw_dot)
    noisy_heading_pub.publish(Float32MultiArray(data = noisy_state))

def turtle_init():

    global noisy_heading_pub
    rospy.init_node('Circular_Path', anonymous= True)

    rospy.Subscriber("/odom", Odometry, odometry_received)
    
    noisy_heading_pub = rospy.Publisher('/noisy_state', Float32MultiArray, queue_size=10)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(2)

    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 1

    while not rospy.is_shutdown():
        velocity_pub.publish(vel)
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    turtle_init()