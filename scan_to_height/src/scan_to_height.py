#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import copy

# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

height_publisher = None
height_scan_publisher = None
clean_scan_publisher = None

# averaging ringbuffer
ringbuf_size = 10
ringbuf_idx = 0
ringbuf = [0] * ringbuf_size
ringbuf_sum = 0.0

def callback(scan):
    global ringbuf, ringbuf_idx, ringbuf_sum

    pts = 35
    begin_idx = len(scan.ranges)-pts
    begin_angle = begin_idx * scan.angle_increment

    # this process (creating an entire copy) is slow: consider ommitting it after tuning
    #height_scan = copy.deepcopy(scan)
    #height_scan.angle_min = height_scan.angle_min + begin_angle
    #height_scan.ranges = height_scan.ranges[begin_idx:]
    #if height_scan.intensities is not []:
    #    height_scan.intensities = height_scan.intensities[begin_idx:]
    #height_scan_publisher.publish(height_scan)

    ## determine height
    # median
    heights = sorted(scan.ranges[begin_idx:])
    height = heights[len(heights) / 2]
    height_publisher.publish(height)
    
    # mean
    ringbuf_idx = (ringbuf_idx + 1) % ringbuf_size
    ringbuf_sum = ringbuf_sum - ringbuf[ringbuf_idx]
    ringbuf[ringbuf_idx] = height
    ringbuf_sum = ringbuf_sum + height
    hp2.publish(ringbuf_sum/ringbuf_size)

    ## create height free scan
    clean_scan = scan
    clean_scan.angle_max = scan.angle_max - pts*scan.angle_increment
    clean_scan.ranges = scan.ranges[0:begin_idx]
    clean_scan.intensities = []
    #if clean_scan.intensities is not []:
    #    clean_scan.intensities = clean_scan.intensities[0:begin_idx]
    clean_scan_publisher.publish(clean_scan)


def listener():
    global height_publisher, hp2, height_scan_publisher, clean_scan_publisher
    height_publisher = rospy.Publisher('height_raw', Float32)
    hp2 = rospy.Publisher('height', Float32)
    height_scan_publisher = rospy.Publisher('height_scan', LaserScan)
    clean_scan_publisher = rospy.Publisher('clean_scan', LaserScan)

    rospy.init_node('scan_to_height', anonymous=False)
    rospy.Subscriber("scan", LaserScan, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()


#def talker():p
#    pub = rospy.Publisher('height', Float32, queue_size=10)
#    rospy.init_node('scan_to_height', anonymous=True)
#    while not rospy.is_shutdown():
#        str = "hello world %s"%rospy.get_time()
#        rospy.loginfo(str)
#        pub.publish(str)
#        r.sleep()
        
#if __name__ == '__main__':
#    try:
#        talker()
#    except rospy.ROSInterruptException: pass
