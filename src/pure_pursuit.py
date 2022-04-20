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
from log_file import LogFile

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic")
        self.lookahead        = 1. #1.2 Meter works pretty well without velocity modulation
        self.speed            = 2.0 #RESET THIS for testing!!!
        self.wrap             = 0
        self.wheelbase_length = 0.325
        self.trajectory  = utils.LineTrajectory("/followed_trajectory")
        self.traj_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.trajectory_callback, queue_size=1)
        self.drive_pub = rospy.Publisher("/drive", AckermannDriveStamped, queue_size=1)
        self.listener=tf.TransformListener()
        self.drive_cmd = AckermannDriveStamped()
        #Subscribe to localization and get my current x and y points in the map frame
        self.traj_message=None
        self.marker_pub = rospy.Publisher("/lookahead_intersection", Marker, queue_size=1)
        self.circle_pub = rospy.Publisher("/car_circle", Marker, queue_size=1)
        self.car_pos = np.array([0,0])
        self.fake_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.fake_cb)

        #logs distances from line over time, plus use final time - initial time to get time to node
        self.ERROR = 1 #flag for turning error logging on or off. If 1, on, 0 off
        if (self.ERROR == 1):
            self.log_error = LogFile("/home/racecar/distancesPPlog1.csv",["distance"])
    def fake_cb (self, msg):
        pass
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
        #TODO Add in and functionality - if both are? i think
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None
        #This version of t finds the closest point on the line eg (we dont need this)
        t = max(0, min(1, - b / (2 * a)))

        return pee1 + t1 * v, pee1 + t2 * v
    def map_to_car_convert(self, result, exp_position, exp_quaternion):
        """
        result = [x,y] from math calc
        exp_position = car position
        exp_quaternion = car quaternion
        """
        translated  = np.array(result) - self.car_pos
        homo_coor = np.concatenate((translated,[exp_position[2]],[1]))
        inverted_quat = tf.transformations.quaternion_inverse(exp_quaternion)
        x, y,_,_ = np.dot(tf.transformations.quaternion_matrix(inverted_quat),homo_coor)
        return x, y

    def draw_marker(self,x,y, id):
        """
        Publish a marker to represent the cone in rviz
        """
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.id = id
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .3
        marker.scale.y = .3
        marker.scale.z = .8
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .1
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.marker_pub.publish(marker)

    def circle(self):
        """
        Publish a marker to represent the cone in rviz
        """
        x,y = self.car_pos
        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = self.lookahead*2
        marker.scale.y = self.lookahead*2
        marker.scale.z = .01
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.circle_pub.publish(marker)

    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print("Receiving new trajectory:", len(msg.poses), "points")
        self.traj_message = msg.poses
        self.trajectory.clear()
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        self.time = rospy.get_time()


    def drive_publish(self):

        """
        Change to base_link_pf to do simulation. Or base_link to do normal
        """
        
        self.listener.waitForTransform("map","base_link_pf", rospy.Time(), rospy.Duration(10.0))
        t = self.listener.getLatestCommonTime("map","base_link_pf")
        exp_position, exp_quaternion = self.listener.lookupTransform("map","base_link_pf", t)
        #The Car Position
        self.car_pos = np.array(exp_position[:2])
        self.circle()
        #Make it a numpy array where first row is x value, second row is y value
        array_of_poses = np.transpose(np.array([[p.position.x, p.position.y] for p in self.traj_message]))
        row_num, col_num = np.shape(array_of_poses)

        #This find the length between all of the segments 
        
        #TODO - Vectorize this!
        segment_lengths=[]
        for col in range(col_num-1):
            dist = self.lineseg_dists(self.car_pos,array_of_poses[:2,col], array_of_poses[:2,col+1])
            segment_lengths.append(dist)
        # closest = np.argmin(segment_lengths)
        closest = np.argmin(np.abs(np.array(segment_lengths) - self.lookahead))
        try:
            result_front, result_back = self.find_int(self.car_pos,self.lookahead,array_of_poses[:2,closest],array_of_poses[:2,closest+1])
            relative_x_front, relative_y_front = self.map_to_car_convert(result_front,exp_position, exp_quaternion )
            relative_x_back, relative_y_back = self.map_to_car_convert(result_back,exp_position, exp_quaternion )
            self.draw_marker(relative_x_front, relative_y_front,1)
            self.draw_marker(relative_x_back, relative_y_back,2)
            self.relative_x, self.relative_y =  relative_x_front, relative_y_front
        except:
            print("Hey there cowboy, the car is offtrack")
            return
        

        #Experimental
        #distnace from car to next seg_end 
        try:
            start_of_next_seg = np.transpose(array_of_poses[:2,closest+int(self.lookahead*4)])
        except:
            start_of_next_seg = np.transpose(array_of_poses[:2,-1])
        
        
        l = np.sqrt(self.relative_x**2 + self.relative_y**2)
        a,b = self.map_to_car_convert(start_of_next_seg,exp_position, exp_quaternion)
        new_l = np.sqrt(a**2 + b**2)
        #Distance to point to drive to
        
        both = np.array([l,new_l])
        self.draw_marker(a, b,3)
        l = both[np.argmin(both)]
        if l==new_l:
            self.relative_x, self.relative_y = a,b



        nu = np.arctan2(self.relative_y, self.relative_x)
        self.drive_cmd.drive.speed = self.speed
        self.drive_cmd.drive.steering_angle = np.arctan(2*self.wheelbase_length*np.sin(nu)/l)

        #Solved an issue where drive angle was -1 --> Stopped car
        angle_threshold = np.clip(abs(self.drive_cmd.drive.steering_angle), -0.36, 0.36)

        #If we are making a sharp turn, might be nice to slow down a bit. 
        if angle_threshold>0.25:
            self.drive_cmd.drive.speed = self.speed*(1.0-angle_threshold)
        
        #Added in way for car to stop at the end
        safety = 0.5
        last_point = np.transpose(array_of_poses[:2,-1])
        if np.linalg.norm(last_point-self.car_pos)<safety:
            self.drive_cmd.drive.speed = 0
        self.drive_pub.publish(self.drive_cmd)

        #log the error
        if (self.ERROR == 1):
             e = self.lineseg_dists(self.car_pos,np.transpose(array_of_poses[:2,np.argmin(segment_lengths)]), np.transpose(array_of_poses[:2, np.argmin(segment_lengths)+1]))
             self.log_error.log(str(rospy.get_time()),[e])

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    r=rospy.Rate(10)
    while not rospy.is_shutdown():
        if pf.traj_message is not None:
            pf.drive_publish()
        # r.sleep()
    # rospy.spin()
