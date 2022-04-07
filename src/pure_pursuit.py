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
        self.lookahead        = 2.
        self.speed            = 0.5
        self.wrap             = 0
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.listener=tf.TransformListener()
        self.drive_cmd = AckermannDriveStamped()
        #Subscribe to localization and get my current x and y points in the map frame

    def lineseg_dists(self, p, a, b):
    # Handle case where p is a single point, i.e. 1d array.
        p = np.atleast_2d(p)

        if np.all(a == b):
            return np.linalg.norm(p - a, axis=1)

        # normalized tangent vector
        d = np.divide(b - a, np.linalg.norm(b - a))

        # signed parallel distance components
        s = np.dot(a - p, d)
        t = np.dot(p - b, d)

        # clamped parallel distance
        h = np.maximum.reduce([s, t, np.zeros(len(p))])

        # perpendicular distance component, as before
        # note that for the 3D case these will be vectors
        c = np.cross(p - a, d)

        # use hypot for Pythagoras to improve accuracy
        return np.hypot(h, c)

    def find_int(self, Q,r,pee1,pee2):
        v = pee2-pee1
        a = np.dot(v,v)
        b = 2 * np.dot(v,pee1 - Q)
        c = np.dot(pee1,pee1) + np.dot(Q,Q) - 2 * np.dot(pee1, Q) - r**2

        disc = b**2 - 4 * a * c
        
        if disc < 0:
            return None
        sqrt_disc = np.sqrt(disc)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)

        #TODO Add in and functionality - if both are? i tink
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None
        
        t = max(0, min(1, - b / (2 * a)))

        return pee1 + t * v
        


    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)


        self.listener.waitForTransform("map","base_link", rospy.Time(), rospy.Duration(10.0))
        t = self.listener.getLatestCommonTime("map","base_link")
        exp_position, exp_quaternion = self.listener.lookupTransform("map","base_link", t)
        car_pos = np.array(exp_position[:2])
        print("The Car is at", car_pos)
        #Make it a numpy array where first row is x value, second row is y value
        array_of_poses = np.transpose(np.array([[p.position.x, p.position.y] for p in msg.poses]))

        row_num, col_num = np.shape(array_of_poses)

        #This find the length between all of the segments 

        #TODO - Vectorize this!
        segment_lengths=[]
        for col in range(col_num-1):
            dist = self.lineseg_dists(car_pos,array_of_poses[:2,col], array_of_poses[:2,col+1])
            segment_lengths.append(dist)
        closest = np.argmin(segment_lengths)
        print("Closest segment is at index", closest)
        print("Closest segment is at position", array_of_poses[:2,closest],array_of_poses[:2,closest+1])
        self.relative_x, self.relative_y = self.find_int(car_pos,self.lookahead,array_of_poses[:2,closest],array_of_poses[:2,closest+1])
        

        # #Distance to point to drive to
        l = np.sqrt(self.relative_x**2 + self.relative_y**2)
        nu = np.arctan2(self.relative_y, self.relative_x)
        self.drive_cmd.drive.speed = self.speed
        self.drive_cmd.drive.steering_angle = np.arctan(2*self.wheelbase_length*np.sin(nu)/l)*l
        self.drive_pub.publish(self.drive_cmd)




if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    rospy.spin()
