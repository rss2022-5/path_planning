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

from visualization_tools import *
import cartography


class Point:
    """ Helper class for points to be able to create PoseArrays
    """
    def __init__(self, u, v):
        self.x = u
        self.y = v

    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class PathPlan:
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        #self.odom_topic = "/odom"
        
        self.map = cartography.Map("/map", 0.2, 0.5)
        # self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)

        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        # self.traj_constructor_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)

        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        ### TWEAKABLE ###

        # The maximum distance between trajectory points (used by the dubins curve configuration generator)
        self.step_size = 0.5
        # The maximum turning radius of the produced solution
        self.turning_radius = 0.75

        # maximum number of samples generated by the RRT algorithm
        self.num_samples = 500
        
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


    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Pull rotation around z
        _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.start_point = (x, y, th)
        

        if not self.start_resolved:
            rospy.loginfo("Start Initialized")
        self.start_resolved = True

    def goal_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        # Pull rotation around z
        _, _, th = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.end_point = (x, y, th)

        if not self.end_resolved:
            rospy.loginfo("End Initialized")
        self.end_resolved = True


        self.plan_path(0,self.end_point,0)
        
    def plan_path(self, start_point, end_point, map):
        if not self.end_resolved or not self.start_resolved or not self.map.resolved:
            return
        rospy.logwarn("Beginning Path Planning")
        
        rate = rospy.Rate(10)
        
        # graph maps poses to [parent, path_to_parent, edge_length, distance]
        graph = {}
        # the root of the tree

        grid = {}
        def add_point(point, data):
            block_coords = self.map.get_block_coords(point)
            grid[block_coords] = point
            graph[point] = data
        
        add_point(self.start_point, [None, None, 0,0])
        
        # Check if a straight line would solve the optimization
        straight_path, path_length = self.steer(self.start_point, self.end_point)
        
        if self.collision_free(straight_path):
            # the straight-line case
            # graph[self.end_point] = [self.start_point, straight_path, path_length, path_length]
            add_point(self.end_point, [self.start_point, straight_path, path_length, path_length])

            rospy.logwarn("Straight line works")
        else:
            rospy.logwarn("Obstacles detected, running RRT")
            # Run a discrete number of samples
            for i in range(self.num_samples):
                # get a random point on the map that is not in collision
                z_rand = tuple( self.map.random_point_clear() )
                # Find nearest vertex to new point
                result = self.nearest(graph, z_rand)
                if result == None:
                    # couldn't find any path back to an existing node
                    continue

                z_nearest, new_path, path_length = result
                
                # End Node: [Initial Node, path]
                parent_distance = graph[z_nearest][3]
                add_point(z_rand, [z_nearest, new_path, path_length, parent_distance+path_length])
                
                # check if the line segment from new vert to goal collides
                end_run, end_length = self.steer(z_rand, self.end_point)
                end_distance = parent_distance+path_length+end_length

                if self.collision_free(end_run) and \
                        (not end_point in graph.keys() or \
                        graph[end_point][3] > end_distance):
                    # rospy.logwarn("Good path to end point")
                    graph[self.end_point] = [z_rand, end_run, end_length, end_distance]
                

                VisualizationTools.plot_tree(graph, self.traj_pub, frame='/map')
                rate.sleep()

        # Search backward for the shortest path
        # Add end point to temp trajectory
        curr = self.end_point
        if not curr in graph.keys():
            rospy.logerr("No path produced by RRT")
            return

        curr_x, curr_y, curr_th = self.end_point
        traj = np.array([[curr_x, curr_y, curr_th]])
        # Iterate backward through parents to find the total path
        while curr != self.start_point:
            # print(self.graph)
            c_parent, c_path, l,total_path = graph[curr]
            # Add elements to the temp trajectory once found
            c_path = np.array(c_path)
            traj = np.concatenate((c_path, traj))
            curr = c_parent

        # Create the trajectory by adding all points
        self.trajectory.clear()
        for el in traj:
            self.trajectory.addPoint(Point(el[0], el[1]))
        
        # visualize trajectory Markers
        self.trajectory.publish_viz()
        
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())


    def steer(self, near, rand):
        """
            returns a smoothed set of poses between two endpoints
            self.step_size is the distance between these interim poses
            self.turning_radius is the max turning radius of the car

            near and rand are both poses in the form (x,y,theta)
        """
        #Use dubins curves
        path = dubins.shortest_path(near, rand, self.turning_radius)
        configurations, _ = path.sample_many(self.step_size)

        
        return (configurations, path.path_length())
    
    def collision_free(self, path):
        # check if each element of the path is in the occupancy grid
        
        for el in path:
            if self.map.is_collision(el):
                return False
        return True

    def collision(self, point):
        u, v = self.real2pix(point)
        return u < 0 or v < 0 or u >= self.width or v >= self.height or self.grid[v, u] ==  1.0
        # return self.grid[v, u] ==  1.0

    def nearest(self, graph, rand):
        assert( len(graph) > 0 )
        
        # find distance of each point in graph to rand point
        gr = graph.keys()
        distances = np.array([])
        lengths = []
        paths = []
        for n in gr:
            path = dubins.shortest_path(n, rand, self.turning_radius)
            configurations, _ = path.sample_many(self.step_size)
            paths.append(configurations)
            lengths.append(path.path_length())

            distances = np.append(distances, path.path_length() + graph[n][3])

        # Use np.argmin to find smallest dist
        # print("near res", gr[np.argmin(paths)])

        for i in np.argsort(distances):
            if self.collision_free(paths[i]):
                return gr[i], paths[i], lengths[i]
        return None
        
                                
if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()
