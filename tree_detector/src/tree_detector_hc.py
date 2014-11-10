#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs import point_cloud2 as pc2
import cv2
import numpy
import math

# we've looked at a MSER matcher and HoughCircles
# I can't get Hough to work at all - only Nones.
# likewise with feature matching
# MSER is slow if you use it over the whole screen
# you can segment it so that you only do it on a region near by a contour
# but it turns out that minimum enclosing circle, while not technically
# particularly accurate, is stable and fairly reliable - MSER doesn't actually
# detect enough.
# This is with min pts = 80.
# for now we just publish min enclosing circle!


# how many pixels per meter?
PIXELS_PER_METER = 50 
# how big?
MAP_SIZE = 2 * 6 * PIXELS_PER_METER

# how many points in an object before we assume it's a tree?
MIN_PTS = 1

# Process 1 out of every freqdivider frames 
FREQ_DIVIDER = 10

pc_publisher = None

# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

precomputed_cos = []
precomputed_sin = []

def precompute(mina, maxa, inca, lena):
    global precomputed_cos, precomputed_sin
    # not sure if this is right?
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
    ## create a copy for visualisation
    #image = cv2.GaussianBlur(image, (7,7), 0)
    origimage = image.copy()
    newimage = numpy.zeros((MAP_SIZE, MAP_SIZE, 1),
                           numpy.uint8)
    #detector = cv2.FeatureDetector_create('MSER')
    # whole image detection
    #fs = detector.detect(image)
    #fs.sort(key = lambda x: -x.size)
    
    #sfs = [x for x in fs if not supress(x,fs)]

    #for f in sfs:
    #    cv2.circle(image, (int(f.pt[0]), int(f.pt[1])), int(f.size/2), (255), 2)

    ret, thresh = cv2.threshold(image,2,255,0)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)

    # feature matching
    #circ_template = numpy.zeros((110, 110,3), numpy.uint8)
    #cv2.circle(circ_template, (55,55), 50, (255,255,255), 3)
    #cd=None
    #cd=cv2.cvtColor(circ_template,cv2.COLOR_BGR2GRAY)
    #cv2.imshow('ct',cd)
    #print cv2.HoughCircles(cd, cv2.HOUGH_GRADIENT, 1, 10)
    #ccs, hierarchy = cv2.findContours(circ_template, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    #c = cv2.convexHull(ccs[0])
    #chm = cv2.HuMoments(cv2.moments(c))

    #for contour in contours:
    #    if len(contour) > MIN_PTS:
    #        # trace the contour to improve hough recog
    #        cv2.polylines(origimage, [contour],False, (128), 1)
    #        cv2.polylines(newimage,[contour],False,(255), 1)
    #        
    #        # elipses are too unstable
    #        #elip= cv2.fitEllipse(contour)
    #        #print("Got tree at " , elip)
    #        #cv2.ellipse(origimage, elip, (255), 2)
    #        
    #        mec = (cv2.minEnclosingCircle(contour))
    #        print(cv2.contourArea(cv2.convexHull(contour)), (3.1415*mec[1]**2)/2)
    #        points += [(mec[0])]
    #        cv2.circle(origimage, (int(mec[0][0]), int(mec[0][1])), int(mec[1]), (127), 2)

    #        # segmented MSER - faster than MSER, but still quite unstable
    #        #x1 = clamp(int(round(mec[0][0]-4*mec[1])),0,MAP_SIZE)
    #        #x2 = clamp(int(round(mec[0][0]+4*mec[1])),0,MAP_SIZE)
    #        #y1 = clamp(int(round(mec[0][1]-4*mec[1])),0,MAP_SIZE)
    #        #y2 = clamp(int(round(mec[0][1]+4*mec[1])),0,MAP_SIZE)
    #        #cv2.rectangle(origimage, (x1,y1), (x2,y2), (127), 2)
    #        #print("trying ", (x1,y1), (x2,y2))
    #        #fs = detector.detect(image[y1:y2, x1:x2])
    #        #cv2.imshow('reg', image[y1:y2,x1:x2])
    #        #fs.sort(key = lambda x: -x.size)
    #
    #        #sfs = [x for x in fs if not supress(x,fs)]
    #
    #        #for f in sfs:
    #        #    print(f.pt)
    #        #    cv2.circle(origimage, (int(f.pt[0]+mec[0][0]-4*mec[1]), int(f.pt[1]+mec[0][1]-4*mec[1])), int(f.size/2), (255), 2)

     #       # more matching
     #       #match = cv2.matchShapes(c, contour, cv2.cv.CV_CONTOURS_MATCH_I2, 0)
     #       #print("It has match: ", match)


    # Hough Circles
    circles = cv2.HoughCircles(image, cv2.cv.CV_HOUGH_GRADIENT, 2, 200, param1=100, param2=10, minRadius=15, maxRadius=25)
    #print(circles)
    if circles is not None:
        circles = numpy.uint16(numpy.around(circles))
        for c in circles[0,:]:
            #print(c)
            cv2.circle(origimage,(c[0],c[1]),c[2],(255),2)
            points+=[(c[0],c[1])]


    #print('--')
    #cv2.drawContours(origimage, contours, -1, (127), 3)
    #small = cv2.resize(origimage, (0,0), fx=0.5, fy=0.5) 
    cv2.imshow('scan', origimage)
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
