#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PointStamped, Point, \
                              QuaternionStamped, Quaternion
from nav_msgs.msg import Odometry

publisher = None
latest_height = 0

def callback(odommsg):
    odommsg.pose.pose.position.z = latest_height

    publisher.publish(odommsg)
    
def height_cb(height):
    global latest_height
    latest_height = height.data


def listener():
    global publisher
    rospy.init_node('flying', anonymous=False)
    rospy.Subscriber("/vo", Odometry, callback)
    rospy.Subscriber("/height", Float32, height_cb)
    publisher = rospy.Publisher("flyingodom", Odometry)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
