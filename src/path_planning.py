#!/usr/bin/env python

import rospy
import dubins
import numpy as np
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
from visualization_msgs.msg import Marker
import rospkg
import time, os
from tf.transformations import quaternion_matrix, euler_from_quaternion
from utils import LineTrajectory

class Point(object):
    def __init__(self, u, v):
        self.x = u
        self.y = v

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.fake_listen = rospy.Subscriber("/trajectory/current", PoseArray, self.fake_cb)
        self.click_listen = rospy.Subscriber("/clicked_point", Marker, self.click_cb)
        ## TWEAKABLE ##
        self.step_size = 1
        self.turning_radius = 0.4

        # Declare vars
        self.resolution = None
        self.width = None
        self.height = None
        self.origin = None
        self.grid = None
        self.map_resolved = False
        self.start_point = None
        self.start_resolved = False
        self.end_point = None
        self.end_resolved = False
        self.graph = None

        self.plan_path(0, 0 , 0)

    def fake_cb(self, msg):
        pass

    def click_cb(self, msg):
        point = msg.pose.position
        print(point.x, point.y)
                                                 
    def map_cb(self, msg):
        data = np.array(msg.data, np.double)/100.
        data = np.clip(data, 0, 1)
        data = np.where(data > 0.3, 1, 0)
        self.resolution = float(msg.info.resolution)
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        # the real world pose of the origin of the map [m, m, rad]
        origin_p = msg.info.origin.position
        origin_o = msg.info.origin.orientation
        origin_o = euler_from_quaternion((origin_o.x, origin_o.y,
                                         origin_o.z, origin_o.w))
        self.origin = (origin_p.x, origin_p.y, origin_o[2])
        self.grid = np.reshape(data, (self.height, self.width))
        self.map_resolved = True

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.start_point = (x, y, th)
        self.start_resolved = True

    def goal_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.end_point = (x, y, th)
        self.end_resolved = True
        
    def plan_path(self, start_point, end_point, map):
        ## CODE FOR PATH PLANNING ##
        self.graph = {}

        # TWEAKABLE CONSTANTS #
        num_samples = 10

        # Wait for initialization
        while self.end_resolved is not True or self.start_resolved is not True or self.map_resolved is not True:
            pass        

        # Check if a straight line would solve the optimization
        straight_path = self.steer(self.start_point, self.end_point)
        if not self.collision_free(straight_path):
            # Run a discrete number of samples
            for i in range(num_samples):
                z_rand = self.sample()
                while self.collision(z_rand):
                    z_rand = self.sample()
                z_nearest = self.nearest(z_rand)
                # Compute path from newest point to nearest one
                new_path = self.steer(z_nearest, z_rand)
                if self.collision_free(new_path):
                    # z_new = new_path(T)
                    # End Node: [Initial Node, path]
                    self.graph[z_rand] = [z_nearest, new_path]
                    # check if the line segment from new vert to goal collides
                    end_run = self.steer(z_rand, self.end_point)
                    # if not, end RRT
                    if self.collision_free(end_run):
                        self.graph[self.end_point] = [z_rand, end_run]
                        break;
        # the straight-line case
        else:
            self.graph[self.end_point] = [self.start_point, straight_path]

        # Search backward for the shortest path
        curr = self.end_point
        curr_x, curr_y, curr_th = self.end_point
        traj = np.array([[curr_x, curr_y, curr_th]])
        while curr != self.start_point:
            c_parent, c_path = self.graph[curr]
            # Add elements to the trajectory once found
            c_path = np.array(c_path)
            traj = np.concatenate((c_path, traj))
            curr = c_parent

        for el in traj:
            self.trajectory.addPoint(Point(el[0], el[1]))
       
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())

        # visualize trajectory Markers
        self.trajectory.publish_viz()

    def sample(self):
        x = np.random.random_sample()*self.width
        y = np.random.random_sample()*self.height
        th = np.random.random_sample()*2*np.pi
        return (x, y, th)

    def steer(self, near, rand):
        #Use dubins curves
        path = dubins.shortest_path(near, rand, self.turning_radius)
        configurations, _ = path.sample_many(self.step_size)
        return configurations
    
    def collision_free(self, path):
        # check if each element of the path is in the occupancy grid
        for el in path:
            if self.collision(el):
                return False
        return True

    def collision(self, point):
        u, v = self.real2pix(point)
        return self.grid[v, u] ==  1.0

    def nearest(self, rand):
        if len(self.graph) == 0:
            return self.start_point
        # find distance of each point in graph to rand point
        gr = np.array([])
        paths = np.array([])
        for n in self.graph:
            gr = np.append(gr, n)
            path_len = dubins.shortest_path(n, rand, self.turning_radius).path_length()
            paths = np.append(paths, path_len)
        # Use np.argmin to find smallest dist
        return gr[np.argmin(paths)]

    def real2pix(self, point):
        x, y, th = point
        # multiply (x, y) * self.resolution
        # apply rotation and translation of self.origin.orientation and self.origin.position
        ang = self.origin[2]
        
        rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        y = -y
        rot = np.append(rot, np.array([[self.origin[0], self.origin[1]]]).T, 1)
        rot = np.append(rot, np.array([[0, 0, 1]]), 0)

        print(rot)
        p_matrix = np.array([[x, y, 1]]).T
        rotated = np.dot(rot, p_matrix)
        rotated = rotated/self.resolution
        return (np.rint(rotated[0]).astype(int), np.rint(rotated[1]).astype(int))
        
        
                                
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()

