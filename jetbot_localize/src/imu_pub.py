#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3
from std_msgs.msg import String

def imu_callback(imu_data):
    imu_msg = Imu()
    mag_msg = MagneticField()
    deg_to_rad = 3.14159/180
    timestamp = rospy.Time.now() 
    imu_msg.header.stamp = timestamp
    mag_msg.header.stamp = timestamp
    frame = "base_link"
    imu_msg.header.frame_id = frame
    mag_msg.header.frame_id = frame
    
    # Extract data from the received string
    imu_values = [float(x) for x in imu_data.data.split(',')]
    
    # Fill in orientation data (assuming IMU doesn't produce orientation estimate)
    imu_msg.orientation_covariance[0] = -1
    
    imu_msg.angular_velocity = Vector3(imu_values[0]*deg_to_rad, imu_values[1]*deg_to_rad, imu_values[2]*deg_to_rad)
    
    imu_msg.linear_acceleration = Vector3(imu_values[3]*10, imu_values[4]*10, imu_values[5]*10)
    
    imu_pub.publish(imu_msg)

    mag_msg.magnetic_field = Vector3(imu_values[6]*10**6, imu_values[7]*10**6, imu_values[8]*10**6)

    mag_pub.publish(mag_msg)

if __name__ == '__main__':
    rospy.init_node('imu_converter', anonymous=True)
    
    imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=10)
    mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size=10)
    rospy.Subscriber('imu_raw', String, imu_callback)
    
    rospy.spin()