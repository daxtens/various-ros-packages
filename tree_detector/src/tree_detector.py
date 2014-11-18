#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2 as pc2
import numpy
import cv2
import math

# it turns out that minimum enclosing circle, while not technically
# particularly accurate, is stable and fairly reliable
# for now we just publish min enclosing circle!

# display the scan with the detected circles for debugging?
DISPLAY_SCAN = False

# how many pixels per meter?
PIXELS_PER_METER = 100 # pix per cm
# how big?
MAP_SIZE = 2 * 6 * PIXELS_PER_METER

# how many points in an object before we assume it's a tree?
MIN_PTS = 80

# Process 1 out of every freqdivider frames 
FREQ_DIVIDER = 10

pc_publisher = None

precomputed_cos = []
precomputed_sin = []

def precompute(mina, maxa, inca, lena):
    global precomputed_cos, precomputed_sin
    angles = numpy.linspace(mina, maxa, num=lena)
    precomputed_cos = [math.cos(a) for a in angles]
    precomputed_sin = [math.sin(a) for a in angles]
    print("precomputed " + str(lena) + " angles")

fd_count = -1
def callback(scan):
    global fd_count
    fd_count+=1
    if not (fd_count % FREQ_DIVIDER == 0):
        return

    points=[]

    ## precompute sines and cosines of angles if needed
    if len(scan.ranges) != len(precomputed_sin):
        precompute(scan.angle_min, scan.angle_max, 
                   scan.angle_increment, len(scan.ranges))

    ## map ranges to x and y
    xs = [r * precomputed_cos[i] for (i,r) in enumerate(scan.ranges)]
    ys = [r * precomputed_sin[i] for (i,r) in enumerate(scan.ranges)]
    
    ## create actual image
    image = numpy.zeros((MAP_SIZE, MAP_SIZE, 1),
                        numpy.uint8)

    for i in range(len(xs)):
        if math.isnan(xs[i]) or math.isnan(ys[i]) or \
           abs(round(xs[i]))*PIXELS_PER_METER >= MAP_SIZE/2 or \
           abs(round(ys[i]))*PIXELS_PER_METER >= MAP_SIZE/2:
            pass
        else:
            image[round(xs[i]*PIXELS_PER_METER)+MAP_SIZE/2,
                  round(ys[i]*PIXELS_PER_METER)+MAP_SIZE/2, 0] = 255

    ## blur to remove noise
    image = cv2.GaussianBlur(image, (11,11), 0)

    ## create a copy for visualisation
    if DISPLAY_SCAN:
        origimage = image.copy()

    ## Threshold the image and detect contours
    ret, thresh = cv2.threshold(image,2,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)


    for contour in contours:
        if len(contour) > MIN_PTS:

            # fit a circle
            mec = (cv2.minEnclosingCircle(contour))

            points += [(mec[0])]

            if DISPLAY_SCAN:
                cv2.circle(origimage, (int(mec[0][0]), int(mec[0][1])), int(mec[1]), (127), 2)

    if DISPLAY_SCAN:
        cv2.drawContours(origimage, contours, -1, (127), 3)
        small = cv2.resize(origimage, (0,0), fx=0.5, fy=0.5)
        cv2.imshow('scan', small)
        cv2.waitKey(1)

    ## convert points into pointcloud
    #print points
    points_xyz = [(float(x-MAP_SIZE/2)/PIXELS_PER_METER,
                   float(y-MAP_SIZE/2)/PIXELS_PER_METER,
                   0) for (y, x) in points]
    cloud = PointCloud2()
    cloud.header.frame_id = "laser" 
    cloud = pc2.create_cloud_xyz32(cloud.header, points_xyz)
    pc_publisher.publish(cloud)


def listener():
    global pc_publisher
    rospy.init_node('tree_detector', anonymous=False)
    rospy.Subscriber("scan", LaserScan, callback)
    pc_publisher = rospy.Publisher("trees", PointCloud2)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
        
if __name__ == '__main__':
    listener()
