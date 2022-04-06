#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = # FILL IN #
        self.speed            = 0.5
        self.wrap             = # FILL IN #
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)


        #Subscribe to localization and get my current x and y points in the map frame

    def dist2(v, w):
    #  return sqr(v.x - w.x) + sqr(v.y - w.y)
        return (np.linalg.norm(v-w))**2

    def distToSegment(p, v, w):
        l2 = self.dist2(v, w)
        if l2 == 0:
            return self.dist2(p, v)
        t = ((p[0] - v[0]) * (w[0] - v[0]) + (p[1] - v[1]) * (w[1] - v[1])) / l2;
        t = np.max(0, np.min(1, t))
        return np.sqrt(self.dist2(p, np.array([v[0] + t * (w[0] - v[0]),v[1] + t * (w[1] - v[1])]))) 

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print "Receiving new trajectory:", len(msg.poses), "points"
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)

        #Make it a numpy array
        array_of_poses = np.array(msg.poses)
        """This Removes the thetas and should look like
        XXXXXXXXXXXX
        YYYYYYYYYYYY
        """
        array_of_poses = np.transpose(array_of_poses[:,0:2])
        row_num, col_num = np.shape(array_of_poses)
        p = np.array([, ,])
        #This find the length between all of the segments 

        #Temporary - will vectorize later 
        segment_lengths = np.array([distToSegment(p,array_of_poses[0,col],array_of_poses[1,col]) for col in range(col_num)])

        #Distance to point to drive to
        l = np.sqrt(self.relative_x**2 + self.relative_y**2)
        #find nu = angle between car and point to drive to, aTan(y/x)
        nu = np.arctan2(self.relative_y, self.relative_x)
        
        #drive angle = the thing we are setting
        if l <= self.parking_distance + 0.2 + self.speed*0.3:
            
        else:
            rospy.logwarn(l)
            drive_cmd.drive.speed = self.speed
            drive_cmd.drive.steering_angle = np.arctan(2*self.wheelbase_length*np.sin(nu)/l)*l

        self.drive_pub.publish(drive_cmd)




if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
