#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import PointStamped, Point, \
                              QuaternionStamped, Quaternion
from sensor_msgs import point_cloud2 as pc2
from nav_msgs.msg import Odometry
import numpy
import cv2
import math
import tf
import itertools
from tf.transformations import quaternion_from_euler

# how far away can two trees be before we decide they're actually
# two trees and not one tree that's moved. Used in early section only.
SAME_TREE_THRESH_MATCH = 3

# how far can a tree be from existing trees after stabilisation
# before we conclude it's a new tree
# should be lower than the other threshold
SAME_TREE_THRESH_DETECT = 3

# If tree trunks move by up to this much we update our static points
# so that the next scan has a lower error.
SAME_TREE_THRESH_UPDATE=0.3

def ptdistance(a, b):
    return distance((a.x,a.y),(b.x,b.y))

def distance((x1,y1),(x2,y2)):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)

# Input: expects Nx3 matrix of points
# Returns R,t
# R = 3x3 rotation matrix
# t = 3x1 column vector

def rigid_transform_3D(A, B):
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = numpy.mean(A, axis=0)
    centroid_B = numpy.mean(B, axis=0)
    
    # centre the points
    AA = A - numpy.tile(centroid_A, (N, 1))
    BB = B - numpy.tile(centroid_B, (N, 1))

    # dot is matrix multiplication for array
    H = numpy.transpose(AA) * BB

    U, S, Vt = numpy.linalg.svd(H)

    R = Vt.T * U.T

    # special reflection case
    if numpy.linalg.det(R) < 0:
       #print "Reflection detected"
       raise Exception
    #   Vt[2,:] *= -1
    #   R = Vt.T * U.T

    t = -R*centroid_A.T + centroid_B.T

    #print t

    return R, t


tf_listener = None
pc_publisher = None
pc2_publisher = None
odom_publisher = None
static_pts = []
# http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html

def callback(trees_scan):
    global static_pts, position, yaw
    tf_broadcaster = tf.TransformBroadcaster()

    # get the scan
    points_xyz = pc2.read_points(trees_scan)
    scan_points = [(x, y) for (x, y, z) in points_xyz]


    if len(scan_points) == 0:
        return


    #print points

    ## get the scan->world transform
    #try:
    #    (trans,rot) = tf_listener.lookupTransform('/world','/base_link', rospy.Time(0))
    #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
    #    print(e)
    #    return
    
    #print('t: ', trans)
    #print('r: ', rot)

    # convert points to world FoR
    # this is probably a terrible way to do this. Oh well.
    scan_pts_in_world = [tf_listener.transformPoint(
                             '/world',
                             PointStamped(trees_scan.header,
                                          Point(p[0],p[1],0)))
                         for p in scan_points]
    

    # attempt to match the points to the static scan
    # what we're really trying to do is get a set of shared points
    # between the static scan and the new scan that we can use to
    # line up the new scan before we detect new trees

    # again, horrible way to do this
    common_sp = []
    common_np = []
    for p in scan_pts_in_world:
        found = False
        mindist = SAME_TREE_THRESH_MATCH
        for sp in static_pts:
            #print(p,sp)
            dist = distance((p.point.x, p.point.y),
                            sp) 
            if dist < mindist:  
                found = True
                bestpoint = sp
                mindist = dist
        
        if found:
            common_sp += [bestpoint]
            common_np += [numpy.array([p.point.x, p.point.y])]
            #print("coundn't find a match for a tree at ", p.point.x, p.point.y)
            #print("could be a new tree")

    ## Match our position and orientation based on common pts
    ## the idea is New = Static*R + t + N
    publish_pose = True
    theta_cov = 1e-6
    if len(common_sp) == 0:
        # we have nothing. Don't publish a pose.
        R = numpy.identity(2)
        t = numpy.matrix([[0],[0]])
        rmse = 0
        publish_pose = False
    elif len(common_sp) == 1:
        # move to be in the same spot:
        (dx,dy) = common_sp[0] - common_np[0]
        R = numpy.identity(2)
        t = numpy.matrix([[dx], [dy]])
        publish_pose = False
        rmse = 0
    elif len(common_sp) >= 2:
        # apply rigid transform
        # http://nghiaho.com/?page_id=671
        # it claims to be 3d but works for 2d too
        staticp = numpy.matrix(common_sp)
        newp = numpy.matrix(common_np)
        try:
            R, t = rigid_transform_3D((newp),
                                      (staticp))
        except:
            return
        
                    
    # now we transform all the world points with R and t
    scan_pts_in_world_xy = numpy.matrix([[p.point.x, p.point.y] 
                                         for p in scan_pts_in_world])

    rd = R*scan_pts_in_world_xy.T
    rtd = rd + numpy.tile(t, (1, len(scan_pts_in_world)))
    corrected_pts_xy = rtd.T

    # also error
    err = corrected_pts_xy - scan_pts_in_world_xy
    err = numpy.multiply(err, err)
    err = numpy.sum(err)/len(corrected_pts_xy)
    rmse = math.sqrt(err)

    # we're occasionally experiencing big jumps
    # if we get one, drop this entire match - don't look
    # for new trees
    normt = numpy.linalg.norm(t)
    yaw = math.asin(R[1,0])
    #print(normt, rmse)
    if normt > 4 or rmse > 4 or yaw > 3.14/3:
        print('dropping', normt, rmse, yaw, t)

    # detect new/moved trees
    for p in numpy.array(corrected_pts_xy):
        for (i,sp) in enumerate(static_pts):
            found = False
            dist = distance(p, sp)
            #if dist < SAME_TREE_THRESH_UPDATE:
            #    static_pts[i] = p
            #    found = True
            #    break
            if dist < SAME_TREE_THRESH_DETECT:
                found = True
                break
        
        if not found:
            print('new tree at: ', p)
            static_pts += [p]

    # publish detected and matched trees
    points_xyz = [(x,y,0) for (x, y) in static_pts]
    cloud = PointCloud2()
    cloud.header.frame_id = "world"
    cloud = pc2.create_cloud_xyz32(cloud.header, points_xyz)
    pc_publisher.publish(cloud)

    points_xyz = [(x,y,0) for (x, y) in 
                  numpy.array(corrected_pts_xy)]
    cloud = PointCloud2()
    cloud.header.frame_id = "world"
    cloud = pc2.create_cloud_xyz32(cloud.header, points_xyz)
    pc2_publisher.publish(cloud)

    # publish the result as odometry (? todo maybe a Pose?)
    if not publish_pose:
        return
    msg = Odometry()
    msg.header.frame_id = '/world'
    #msg.child_frame_id='/base_link'
    msg.header.stamp = rospy.Time.now()
    # where are we in the map frame? transform 0,0 to find out
    abs_pos = tf_listener.transformPoint(
        '/world',
        PointStamped(trees_scan.header,
                     Point(0,0,0)))

    orientation = tf_listener.transformQuaternion(
        '/world',
        QuaternionStamped(trees_scan.header,
                          Quaternion(*quaternion_from_euler(0,0,yaw))))

    msg.pose.pose.position = abs_pos.point
    msg.pose.pose.orientation = orientation.quaternion
    msg.pose.covariance = [1, 0,    0,     0,     0,     0,
                           0,    1, 0,     0,     0,     0,
                           0,    0,    99999, 0,     0,     0,
                           0,    0,    0,     99999, 0,     0,
                           0,    0,    0,     0,     99999, 0,
                           0,    0,    0,     0,     0,     1e-2]

    odom_publisher.publish(msg)


def listener():
    global pc_publisher,pc2_publisher, tf_listener, odom_publisher
    rospy.init_node('tree_fix', anonymous=False)
    rospy.Subscriber("trees", PointCloud2, callback)
    pc_publisher = rospy.Publisher("static_trees", PointCloud2)
    pc2_publisher = rospy.Publisher("corrected_trees", PointCloud2)
    tf_listener = tf.TransformListener()
    odom_publisher = rospy.Publisher("odom", Odometry)
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
