#!/usr/bin/env python

import rospy
import numpy as np
import time
import utils
import tf
import traceback

from geometry_msgs.msg import PoseArray, PoseStamped
from visualization_msgs.msg import Marker
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry

class PurePursuit(object):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """
    def __init__(self):
        self.odom_topic       = rospy.get_param("~odom_topic", "/odom")
        # self.car_frame        = "base_link_pf"
        self.car_frame        = "base_link" # TODO: Link to rosparam
        self.lookahead        = 2. #1.2 Meter works pretty well without velocity modulation
        self.stop_distance    = 0.5 # Make sure this is less than lookahead!
        # self.lookahead        = 2 #1.2 Meter works pretty well without velocity modulation
        self.speed            = 2.0
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

    def lineseg_dists(self, p, a, b):
        # Handle case where p is a single point, i.e. 1d array.
        # a and b are endpoints
        # returns the minimum distance from the point to the line
        # (as if the line were infinite)

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

    def segment_closest_point(self, p, a, b):
        # Similar to lineseg_dists, but catches case where the closest point on
        # the infinite line is not on the line segment
        # TODO: this uhh doesnt work
        p = np.atleast_2d(p)
        a = np.array(a)
        b = np.array(b)
        # catch degenerate line segment-- closest point is the only point
        if np.all(a == b):
            return a

        # Code copied from uhh stack overflow
        # https://stackoverflow.com/questions/28931007/how-to-find-the-closest-point-on-a-line-segment-to-an-arbitrary-point
        d = np.array(b - a)

        d2 = np.dot(d, d.T)
        # Project point onto the line (will only have an x coordinate in this 1D space)
        nx = ((p - a) * d ) / d2
        
        # clamp the x coordinate to [0,1) too bind it to the line segment
        nx = np.clip(nx, 0, 1)
        
        #convert back to real-world space
        return a + d*nx

    def segment_min_distance(self, p, a, b):
        return np.linalg.norm( self.segment_closest_point(p,a,b) - p, axis=1 )
        

    def segment_max_distance(self,p, a, b):
        # Find the distance from the car to both endpoints and return whichever is greater
        # x1, y1 = self.map_to_car_convert(a, self.exp_position, self.exp_quaternion)
        l1 = np.linalg.norm(a-p, axis=1)

        # x2, y2 = self.map_to_car_convert(a, self.exp_position, self.exp_quaternion)
        l2 = np.linalg.norm(b-p, axis=1)

        return max(l1, l2)

    def find_int(self, Q, r, pee1, pee2):
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
        # TODO Add in and functionality - if both are? i think
        if not (0 <= t1 <= 1 or 0 <= t2 <= 1):
            return None
        # This version of t finds the closest point on the line eg (we dont need this)
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

    def drive_stop(self):
        """
        Stop the car
        """
        self.drive_cmd.drive.speed = 0
        self.drive_pub.publish(self.drive_cmd)
    
    def drive_toward_point(self, point):
        """
        Textbook implementation of Pure Pursuit

        This function includes any slowing down or fancy business we want to do
        This would also be a good place to modulate the lookahead distance
        """

        relative_x, relative_y = self.map_to_car_convert(point,self.exp_position, self.exp_quaternion )


        # This number should always equal lookahead distance
        l = np.sqrt(relative_x**2 + relative_y**2)        

        # get relative angle to target intersection
        nu = np.arctan2(relative_y, relative_x)

        delta = np.arctan(2*self.wheelbase_length*np.sin(nu)/l)
        
        self.drive_cmd.drive.speed = self.speed
        self.drive_cmd.drive.steering_angle = delta
        # if abs(delta)>0.13:
        #     self.drive_cmd.drive.speed = self.speed*(1.0-angle_threshold)
        self.drive_pub.publish(self.drive_cmd)

    def fetch_position(self):
        # Get and plot positions
        self.listener.waitForTransform("map",self.car_frame, rospy.Time(), rospy.Duration(10.0))
        t = self.listener.getLatestCommonTime("map",self.car_frame)
        self.exp_position, self.exp_quaternion = self.listener.lookupTransform("map",self.car_frame, t)
        #The Car Position
        self.car_pos = np.array(self.exp_position[:2])
        self.circle()

    def find_intersection(self):
        #Make it a numpy array where first row is x value, second row is y value
        array_of_poses = np.array([[p.position.x, p.position.y] for p in self.traj_message]).T
        row_num, col_num = np.shape(array_of_poses)

        #This find the length between all of the segments         
        #TODO - Vectorize this!
        segment_lengths=[]

        for col in range(col_num-1):
            dist = self.lineseg_dists(self.car_pos,array_of_poses[:2,col], array_of_poses[:2,col+1])
            segment_lengths.append(dist)
        
        if len(segment_lengths) == 0:
            rospy.logwarn("Empty Trajectory")
            return None
        # index of point beginning the closest segment
        closest = np.argmin(np.abs(np.array(segment_lengths) - self.lookahead))

        try:
            # find intersctions on the single closest line segment
            result = self.find_int(self.car_pos,self.lookahead,array_of_poses[:2,closest],array_of_poses[:2,closest+1])

            if result == None:
                rospy.logwarn("No intersection found")
                return None

            result_front, result_back = result

            # plotting intersections
            relative_x_front, relative_y_front = self.map_to_car_convert(result_front,self.exp_position, self.exp_quaternion )
            relative_x_back, relative_y_back = self.map_to_car_convert(result_back,self.exp_position, self.exp_quaternion )
            self.draw_marker(relative_x_front, relative_y_front,1)
            self.draw_marker(relative_x_back, relative_y_back,2)
            
            # save for future use
            self.relative_x, self.relative_y =  relative_x_front, relative_y_front
        except Exception as e:
            rospy.logerr(traceback.format_exc())
            rospy.logerr(e)
            rospy.logwarn("Hey there cowboy, the car is offtrack")
            return None

        return result_front

    def find_intersections_all(self):
        # Make it a numpy array where first row is x value, second row is y value
        array_of_poses = np.array([[p.position.x, p.position.y] for p in self.traj_message]).T
        row_num, col_num = np.shape(array_of_poses)

        for col in range(col_num-1):
            dist = self.segment_min_distance(self.car_pos,array_of_poses[:2,col], array_of_poses[:2,col+1])
            dist = self.segment_max_distance(self.car_pos,array_of_poses[:2,col], array_of_poses[:2,col+1])
            segment_lengths.append(dist)
        
        if len(segment_lengths) == 0:
            rospy.logwarn("Empty Trajectory")
            return None
        # index of point beginning the closest segment

    def drive_publish(self):
        """
        Change to base_link_pf to do simulation. Or base_link to do normal
        """

        # retrieve and store local position of the car
        self.fetch_position()

        # Find distance to endpoint
        x, y = self.map_to_car_convert(self.trajectory.points[-1], self.exp_position, self.exp_quaternion)
        l = np.sqrt(x**2 + y**2)

        # Stop if we're close
        if l < self.stop_distance:
            self.drive_stop()
            return
        
        # Else, find nearest intersection and get there
        target_point = self.find_intersections_all()
        if not target_point is None:
            self.drive_toward_point(target_point)

if __name__=="__main__":
    rospy.init_node("pure_pursuit")
    pf = PurePursuit()
    # r=rospy.Rate(20)
    while not rospy.is_shutdown():
        if pf.traj_message is not None:
            pf.drive_publish()
        # r.sleep()
    # rospy.spin()
