#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu 
import numpy as np 
import tf

def odometry_received(msg):
    global noisy_state, x_robot, A, P_0, Q, R, C, I, yaw_imu, yaw_dot_imu, robot_filtered_x

    explicit_quat = [msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w]

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

    x_robot = []
    x_robot.append(yaw_imu)
    x_robot.append(yaw_dot_imu)
    P = (A.dot(P_0).dot(A.T))+Q

    K = P.dot((C.T)).dot(np.reciprocal((C.dot(P).dot((C.T)))+R)) # Kalman Gain
    x_robot = x_robot + K.dot((noisy_state - C.dot(x_robot))) # correction of estimate
    P = (I - K.dot(C)).dot(P).dot((I - K.dot(C)).T) + K.dot(R).dot((K.T)) # corrrection of uncertainty
    P_0 = P
    
    robot_filtered_x = []
    robot_filtered_x.append(x_robot[0])
    robot_filtered_x.append(x_robot[1])

    filtered_angle_pub.publish(Float32MultiArray(data = robot_filtered_x))
    rate.sleep()

def imu_recieved(msg):
    global yaw_imu, yaw_dot_imu
    
    quat = [msg.orientation.x,
    msg.orientation.y, msg.orientation.z,
    msg.orientation.w]
    roll_imu, pitch_imu, yaw_imu = tf.transformations.euler_from_quaternion(quat)
    yaw_dot_imu = msg.angular_velocity.z
    rate.sleep()

def turtle_init():
    global filtered_angle_pub, noisy_heading_pub, A, P_0, Q, R, C, I, rate
    rospy.init_node('Estimate_Orientation')

    I = np.eye(2)
    dt = .001
    sigma_p = 0.25
    sigma_v = 0.3
    sigma_y = 0.2

    A =np.array([[1, dt],
                 [0, 1]])

    P_0 =np.array([[500, 0],
                   [0, 500]])

    Q = np.array([[sigma_p**2, 0],
                  [0, sigma_v**2]])
                  
    x0 = np.array([0.0, 0.0])

    R = 0
    # R = np.array([[sigma_y**2, 0],
      #           [0, sigma_y**2]])

    C = np.array([1, 0])
    #C = np.array([[1, 0],
              #    [0, 1]])


    rospy.Subscriber("/odom", Odometry, odometry_received)
    rospy.Subscriber("/imu", Imu, imu_recieved)
    filtered_angle_pub = rospy.Publisher('/filtered_heading', Float32MultiArray, queue_size=10)
    noisy_heading_pub = rospy.Publisher('/noisy_state', Float32MultiArray, queue_size=10)
    velocity_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(10)

    vel = Twist()
    vel.linear.x = 0.5
    vel.angular.z = 1

    while not rospy.is_shutdown():
        velocity_pub.publish(vel)
        rate.sleep()
    
    rospy.spin()

if __name__ == '__main__':
    turtle_init()