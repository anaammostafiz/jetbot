#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import tf2_ros
import math

# Create odometry data publishers
odom_data_pub = rospy.Publisher('odom_data_euler', Odometry, queue_size=100)
odom_data_pub_quat = rospy.Publisher('odom_data_quat', Odometry, queue_size=100)
odomNew = Odometry()
odomOld = Odometry()

# Initial pose
initialX = 0.0
initialY = 0.0
initialTheta = 0.00000000001
PI = 3.141592

# Robot physical constants
TICKS_PER_REVOLUTION = 330
WHEEL_RADIUS = 0.03375
WHEEL_BASE = 0.183
TICKS_PER_METER = 1556.18

# Distance both wheels have traveled
distanceLeft = 0.0
distanceRight = 0.0

# Flag to see if initial pose has been received
initialPoseRecieved = False

# Callback function to get initial_2d message
def set_initial_2d(rvizClick):
    global odomOld, initialPoseRecieved
    odomOld.pose.pose.position.x = rvizClick.pose.pose.position.x
    odomOld.pose.pose.position.y = rvizClick.pose.pose.position.y
    odomOld.pose.pose.orientation.z = rvizClick.pose.pose.orientation.z
    initialPoseRecieved = True

# Callback function to calculate distance the left wheel has traveled
def calc_left(leftCount):
    global distanceLeft
    lastCountL = getattr(calc_left, 'lastCountL', 0)
    if leftCount.data != 0 and lastCountL != 0:
        leftTicks = leftCount.data - lastCountL
        if leftTicks > 10000:
            leftTicks = 0 - (65535 - leftTicks)
        elif leftTicks < -10000:
            leftTicks = 65535 - leftTicks
        distanceLeft = leftTicks / TICKS_PER_METER
    setattr(calc_left, 'lastCountL', leftCount.data)

# Callback function to calculate distance the right wheel has traveled
def calc_right(rightCount):
    global distanceRight
    lastCountR = getattr(calc_right, 'lastCountR', 0)
    if rightCount.data != 0 and lastCountR != 0:
        rightTicks = rightCount.data - lastCountR
        if rightTicks > 10000:
            distanceRight = (0 - (65535 - distanceRight)) / TICKS_PER_METER
        elif rightTicks < -10000:
            rightTicks = 65535 - rightTicks
        distanceRight = rightTicks / TICKS_PER_METER
    setattr(calc_right, 'lastCountR', rightCount.data)

# Function to publish a nav_msgs::Odometry message in quaternion format
def publish_quat():
    global odomNew, odom_data_pub_quat
    q = quaternion_from_euler(0, 0, odomNew.pose.pose.orientation.z)
    quatOdom = Odometry()
    quatOdom.header.stamp = odomNew.header.stamp
    quatOdom.header.frame_id = "odom"
    quatOdom.child_frame_id = "base_link"
    quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x
    quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y
    quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z
    quatOdom.pose.pose.orientation.x = q[0]
    quatOdom.pose.pose.orientation.y = q[1]
    quatOdom.pose.pose.orientation.z = q[2]
    quatOdom.pose.pose.orientation.w = q[3]
    quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x
    quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y
    quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z
    quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x
    quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y
    quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z

    for i in range(36):
        if i == 0 or i == 7 or i == 14:
            quatOdom.pose.covariance[i] = 0.01
        elif i == 21 or i == 28 or i == 35:
            quatOdom.pose.covariance[i] += 0.1
        else:
            quatOdom.pose.covariance[i] = 0

    odom_data_pub_quat.publish(quatOdom)

# Function to update odometry information
def update_odom():
    global odomNew, odomOld, distanceLeft, distanceRight
    cycleDistance = (distanceRight + distanceLeft) / 2
    cycleAngle = math.asin((distanceRight - distanceLeft) / WHEEL_BASE)
    avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z

    if avgAngle > PI:
        avgAngle -= 2 * PI
    elif avgAngle < -PI:
        avgAngle += 2 * PI

    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + math.cos(avgAngle) * cycleDistance
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + math.sin(avgAngle) * cycleDistance
    odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z

    if math.isnan(odomNew.pose.pose.position.x) or math.isnan(odomNew.pose.pose.position.y) or math.isnan(odomNew.pose.pose.position.z):
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y
        odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z

    if odomNew.pose.pose.orientation.z > PI:
        odomNew.pose.pose.orientation.z -= 2 * PI
    elif odomNew.pose.pose.orientation.z < -PI:
        odomNew.pose.pose.orientation.z += 2 * PI

    odomNew.header.stamp = rospy.Time.now()
    time_diff = (odomNew.header.stamp - odomOld.header.stamp).to_sec()

    if time_diff != 0:
        odomNew.twist.twist.linear.x = cycleDistance / time_diff
        odomNew.twist.twist.angular.z = cycleAngle / time_diff

    odomOld.pose.pose.position.x = odomNew.pose.pose.position.x
    odomOld.pose.pose.position.y = odomNew.pose.pose.position.y
    odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z
    odomOld.header.stamp = odomNew.header.stamp

    odom_data_pub.publish(odomNew)

# Main function
def main():
    global odomNew, odomOld, distanceLeft, distanceRight, initialPoseRecieved

    rospy.init_node('odom_pub')
    rate = rospy.Rate(30)

    # Subscribe to ROS topics
    rospy.Subscriber('pos_R', Int16, calc_right, tcp_nodelay=True)
    rospy.Subscriber('pos_L', Int16, calc_left, tcp_nodelay=True)
    rospy.Subscriber('initialpose', PoseWithCovarianceStamped, set_initial_2d)

    # Publisher of simple odom message where orientation.z is an euler angle
    odom_data_pub = rospy.Publisher('odom_data_euler', Odometry, queue_size=100)

    # Publisher of full odom message where orientation is quaternion
    odom_data_pub_quat = rospy.Publisher('odom', Odometry, queue_size=100)

    while not rospy.is_shutdown():
        if initialPoseRecieved:
            update_odom()
            publish_quat()
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
