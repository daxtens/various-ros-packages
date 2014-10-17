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
from tf.transformations import quaternion_from_euler

# how far away can two trees be before we decide they're actually
# two trees and not one tree that's moved. Used in early section only.
SAME_TREE_THRESH_MATCH = 2

# how far can a tree be from existing trees after stabilisation
# before we conclude it's a new tree
SAME_TREE_THRESH_DETECT = 2

# If tree trunks move by up to this much we update our static points
# so that the next scan has a lower error.
SAME_TREE_THRESH_UPDATE=0.3

# the angular window either side of 0 to consider
SCAN_MATCH_WINDOW = 30 * 3.1415/180
SCAN_MATCH_PTS = 360


def ptdistance(a, b):
    return distance((a.x,a.y),(b.x,b.y))


def distance((x1,y1),(x2,y2)):
    return math.sqrt((x1-x2)**2+(y1-y2)**2)


def error(A, B):
    err = A - B
    err = numpy.multiply(err, err)
    err = numpy.sum(err)/len(A)
    return math.sqrt(err)


def rototransform2D(A, B):
    # try a range of angles around 0 to find the best fit
    assert len(A) == len(B)

    N = A.shape[0]; # total points

    centroid_A = numpy.mean(A, axis=0)
    centroid_B = numpy.mean(B, axis=0)
    
    # centre the points
    AA = A - numpy.tile(centroid_A, (N, 1))
    BB = B - numpy.tile(centroid_B, (N, 1))

    # try a set of rotations
    minerr = 999999999
    best_yaw = 0
    for yaw in numpy.linspace(-SCAN_MATCH_WINDOW, SCAN_MATCH_WINDOW,
                              SCAN_MATCH_PTS):
        R = numpy.matrix([[math.cos(yaw), -math.sin(yaw)],
                          [math.sin(yaw), math.cos(yaw)]])
        B2 = BB * R
        err = error(B2, AA)
        if err < minerr:
            minerr = err
            best_yaw = yaw

    yaw = best_yaw
    R = numpy.matrix([[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]])
    t = -R*centroid_A.T + centroid_B.T

    return R, t
    

tf_listener = None
tf_publisher = None
pc_publisher = None
pc2_publisher = None
odom_publisher = None
static_pts = []
last_pos = None
last_yaw = None

world_frame = '/world'

def callback(trees_scan):
    global static_pts, last_pos, last_yaw, tf_publisher

    # get the scan
    points_xyz = pc2.read_points(trees_scan)
    scan_points = [(x, y) for (x, y, z) in points_xyz]

    if len(scan_points) == 0:
        tf_publisher.sendTransform(last_pos,
                                   quaternion_from_euler(0,0,last_yaw),
                                   rospy.Time.now(),
                                   '/odom',
                                   world_frame)
        return

    # convert points to uncorrected world (odom) FoR
    # this is probably a terrible way to do this. Oh well.
    scan_pts_in_world = [tf_listener.transformPoint(
                             '/odom',
                             PointStamped(trees_scan.header,
                                          Point(p[0],p[1],0)))
                         for p in scan_points]
    

    # preconvert the points with the last transform
    # it was probably pretty close; we'll only have to find the 
    # difference between it and the new correction, rather than
    # having to find the complete new correction.
    R = numpy.matrix([[math.cos(last_yaw), -math.sin(last_yaw)],
                      [math.sin(last_yaw),  math.cos(last_yaw)]])
    t = numpy.matrix([[last_pos[0]],[last_pos[1]]])
    scan_pts_in_world_xy = numpy.matrix([[p.point.x, p.point.y] 
                                         for p in scan_pts_in_world])
    rotated = R*scan_pts_in_world_xy.T
    translated = rotated + numpy.tile(t, (1, len(scan_pts_in_world)))
    scan_pts_in_world_xy = numpy.array(translated.T)
                                          
    # attempt to match the points to the static scan
    # what we're really trying to do is get a set of shared points
    # between the static scan and the new scan that we can use to
    # line up the new scan before we detect new trees

    # again, horrible way to do this
    common_sp = []
    common_np = []
    used = [False] * len(static_pts)
    for p in scan_pts_in_world_xy:
        found = False
        mindist = SAME_TREE_THRESH_MATCH
        for (i, sp) in enumerate(static_pts):
            #print(p,sp)
            dist = distance(p, sp)
            if dist < mindist and used[i]:
                print('already used sp', i)
            if dist < mindist and not used[i]:  
                found = True
                bestpoint = sp
                mindist = dist
                used[i] = True
        
        if found:
            common_sp += [bestpoint]
            common_np += [p]

    ## Match our position and orientation based on common pts
    ## the idea is New = Static*R + t + N
    publish_pose = True
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
        # do a rotation + translation
        # we initially did a rigid transform
        # per http://nghiaho.com/?page_id=671
        # but it has issue with reflections
        # so now we just brute force a bunch of different translations
        staticp = numpy.matrix(common_sp)
        newp = numpy.matrix(common_np)
        R, t = rototransform2D(newp, staticp)
        
                    
    # now we transform all the world points with R and t
    scan_pts_in_world_xy = numpy.matrix(scan_pts_in_world_xy)

    rd = R*scan_pts_in_world_xy.T
    rtd = rd + numpy.tile(t, (1, len(scan_pts_in_world)))
    corrected_pts_xy = rtd.T

    # publish detected and matched trees
    points_xyz = [(x,y,0) for (x, y) in static_pts]
    cloud = PointCloud2()
    cloud.header.frame_id = world_frame
    cloud = pc2.create_cloud_xyz32(cloud.header, points_xyz)
    pc_publisher.publish(cloud)

    points_xyz = [(x,y,0) for (x, y) in 
                  numpy.array(corrected_pts_xy)]
    cloud = PointCloud2()
    cloud.header.frame_id = world_frame
    cloud = pc2.create_cloud_xyz32(cloud.header, points_xyz)
    pc2_publisher.publish(cloud)


    # also error
    if len(common_np) >= 2:
        rd = R*numpy.matrix(common_np).T
        rtd = rd + numpy.tile(t, (1, len(common_np)))
        corrected_common_new_pts = rtd.T
        rmse = error(corrected_common_new_pts,
                     numpy.matrix(common_sp))

        rmse_uncorr = error(numpy.matrix(common_np),
                            numpy.matrix(common_sp))
        #print(rmse, rmse_uncorr)
        if rmse > rmse_uncorr:
            print(rmse,rmse_uncorr)

    ## we're occasionally experiencing big jumps
    ## if we get one, drop this entire match
    normt = numpy.linalg.norm(t)
    yaw = math.atan2(R[1,0],R[0,0])
    #print(normt, rmse)
    #if normt > 10 or rmse > 4 or yaw > 3.4/3:
    drop=False
    if normt > 10:
        print('n')
        drop = True
    if rmse > 10:
        print('e')
        drop = True
    if abs(yaw) > 3.14/2:
        print('y')
        drop = True
    if drop:
        tf_publisher.sendTransform(last_pos,
                                   quaternion_from_euler(0,0,last_yaw),
                                   rospy.Time.now(),
                                   '/odom',
                                   world_frame)
        #print('d', rospy.Time.now()-start)
        return

    # detect new/moved trees
    for p in numpy.array(corrected_pts_xy):
        for (i,sp) in enumerate(static_pts):
            found = False
            dist = distance(p, sp)
            if dist < SAME_TREE_THRESH_UPDATE:
                static_pts[i] = p
                found = True
                break
            if dist < SAME_TREE_THRESH_DETECT:
                found = True
                break
        
        if not found:
            print('new tree at: ', p)
            static_pts += [p]

    # publish the result as odometry & transform
    if not publish_pose:
        tf_publisher.sendTransform(last_pos,
                                   quaternion_from_euler(0,0,last_yaw),
                                   rospy.Time.now(),
                                   "/odom",
                                   world_frame)
        #print('p', rospy.Time.now()-start)
        return

    msg = Odometry()
    msg.header.frame_id = 'odom'
    #msg.child_frame_id='/base_link'
    msg.header.stamp = rospy.Time.now()
    # where are we in the map frame? transform (0,0)+t to find out
    abs_pos = tf_listener.transformPoint(
        world_frame,
        PointStamped(trees_scan.header,
                     Point(t[0],t[1],0)))

    orientation = tf_listener.transformQuaternion(
        'odom',
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
    
    # TF hierarchy (per REP 105, iirc) is:
    #     map -> odom -> base_link
    # we want to publish a transform that is map -> odom
    # such that (map -> odom) + (odom -> baselink) is correct
    # we know that the offset that baselink requires is t, R
    # so we publish odom to be at (t.x, t.y) and (0,0,yaw)
    # which will give (map -> odom) + (odom->baselink)
    # equal to (map->uncorrected baselink+(t,R))

    # average the current and the last
    # reduce the effect of large slightly wrong jumps
    new_R = (numpy.matrix([[math.cos(last_yaw), -math.sin(last_yaw)],
                           [math.sin(last_yaw), math.cos(last_yaw)]]) *
             numpy.matrix([[math.cos(yaw), -math.sin(yaw)],
                           [math.sin(yaw), math.cos(yaw)]]))
    new_yaw = math.atan2(new_R[1,0], new_R[0,0])
    new_pos = (last_pos[0] + t[0,0], last_pos[1] + t[1,0],0)

    last_pos = ((last_pos[0]+new_pos[0])/2,
                (last_pos[1]+new_pos[1])/2,
                0)
    last_yaw = (new_yaw+last_yaw)/2
    #last_pos = new_pos
    #last_yaw = new_yaw
    tf_publisher.sendTransform(last_pos,
                               quaternion_from_euler(0,0,last_yaw),
                               rospy.Time.now(),
                               "/odom",
                               world_frame)
    #print('e', rospy.Time.now()-start)


def listener():
    global pc_publisher,pc2_publisher, tf_listener, odom_publisher, \
        tf_publisher, last_pos, last_yaw
    rospy.init_node('tree_fix', anonymous=False)
    rospy.Subscriber("trees", PointCloud2, callback)
    pc_publisher = rospy.Publisher("static_trees", PointCloud2)
    pc2_publisher = rospy.Publisher("corrected_trees", PointCloud2)
    tf_listener = tf.TransformListener()
    odom_publisher = rospy.Publisher("odom", Odometry)
    tf_publisher = tf.TransformBroadcaster()
    last_pos = (0,0,0)
    last_yaw = 0
    tf_publisher.sendTransform(last_pos,
                               quaternion_from_euler(0,0,last_yaw),
                               rospy.Time.now(),
                               "/odom",
                               world_frame)
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
