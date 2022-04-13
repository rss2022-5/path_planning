#!/usr/bin/env python

from operator import truediv
import rospy
import numpy as np
import cv2 as cv
from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Odometry, OccupancyGrid
import rospkg
import time, os
from utils import LineTrajectory
from tf.transformations import quaternion_matrix, euler_from_quaternion
import cartography
import math
from log_file import LogFile

class GridSquare:

    def __init__(self, x, y):
        #Stores pixel coordinates of grid square
        self.x = x
        self.y = y

    def __eq__(self, obj):
        return isinstance(obj, GridSquare) and self.x==obj.x and self.y==obj.y
    
    def __str__(self):
        return "Gridsquare: ("+self.x+","+self.y+")"

    def occupied(self, map):
        if (map[self.y,self.x]==1):
            return True
        else:
            return False

class PathPlan(object):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """
    def __init__(self):
        print("Initializing")
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        self.fake_sub = rospy.Subscriber("/trajectory/current", PoseArray, self.fake_cb)
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

        self.ERROR = 1 #flag for turning error logging on or off. If 1, on, 0 off
        
        if (self.ERROR == 1):
        #      self.log_trajectories = LogFile("/home/racecar/trajectory_A_star_log1-1.csv")
             #1-2 : Traj 1, Log 2
             self.log_distances = LogFile("/home/racecar/distance_log_AStar_1-1.csv",["distances"])
        self.publish_path()
        
        

    def fake_cb (self, msg):
        pass

    def map_cb (self, msg):
        print("call map_cb")
        #Prepare map representation
        map = np.array(msg.data, np.double)
        map = np.where(map < 0, 100, map)
        map = map/100.0
        map = np.clip(map, 0, 1)
        map = np.where(map > 0.5, 1, 0)
        #TODO: Check kernel size
        # kernel = np.ones((5,5),np.uint8)
        # map = cv.erode(map, kernel, iterations = 1)
        # map = cv.dilate(map, kernel, iterations = 1)
        #Storing values
        self.resolution = float(msg.info.resolution)
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.map = np.reshape(map, (self.height, self.width))
        # the real world pose of the origin of the map [m, m, rad]
        origin_p = msg.info.origin.position
        origin_o = msg.info.origin.orientation
        origin_o = euler_from_quaternion((origin_o.x, origin_o.y,
                                         origin_o.z, origin_o.w))
        self.origin = np.array((origin_p.x, origin_p.y))
        self.rotation = origin_o[2]
        print("End map callback")
        self.map_resolved = True

    def odom_cb(self, msg):
        #print("call odom_cb")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.start_point = self.get_pixel((x, y, th))
        self.start_resolved = True

    def goal_cb(self, msg):
        print("call goal_cb")
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.end_point = self.get_pixel((x, y, th))
        self.end_resolved = True

    def get_pixel(self, point):
        # cast to np array
        point = np.array(point[:2])

        # Move to map frame
        x_map_to_point = point - self.origin
        
        ang = -self.rotation
        rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        
        rotated_point = np.dot(rot, x_map_to_point.T)

        # Scale by resolution
        return np.round((rotated_point/self.resolution)).astype(int)
    
    def get_point(self, pixel):
        pixel = np.array(pixel)

        ang = self.rotation
        rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])

        # Undo scaling
        # Apply rotation
        p = np.dot(rot, pixel.T * self.resolution)
        
        # Shift by origin position
        return p + self.origin

    def g(self, map_rep, node1, node2):
        """
        Returns distance between two nodes
        """
        return 1

    def heuristic(self, map_rep, source_id, node2):
        """
        Returns the heuristic for a node
        """
        return math.sqrt((source_id[0]-node2[0])**2+(source_id[1]-node2[1])**2)
    
    def generate_adj_nodes(self, map_rep, source):
        #TODO Modify
        adj = []
        try:
            g = GridSquare(source[0]-1, source[1])
           #print("Made gridsquare object")
            if (not g.occupied(map_rep)):
                adj.append((source[0]-1, source[1]))
            #print("g not occupied")
        except: pass
        try:
            g = GridSquare(source[0]+1, source[1])
            if (not g.occupied(map_rep)):
                adj.append((source[0]+1, source[1]))
        except: pass
        try:
            g = GridSquare(source[0], source[1]-1)
            if (not g.occupied(map_rep)):
                adj.append((source[0], source[1]-1))
        except: pass
        try:
            g = GridSquare(source[0], source[1]+1)
            if (not g.occupied(map_rep)):
                adj.append((source[0], source[1]+1))
        except: pass
        return adj
    
    def next_id(self, map_rep, min_f, open, prev_id, node2):
        """
        Return the node with the next shortest f (distance + heuristic)
        """
        min = float("infinity")
        id = None
        for k in open:
            if (min_f[k]+self.heuristic(map_rep, prev_id, node2)<min):
                id = k
                min = min_f[k]+self.heuristic(map_rep, prev_id, node2)
        return id
    
        
    def a_star(self, map_rep, start, end):
        """
        Return the shortest path between the two nodes

        Parameters:
            map_rep: the result of calling build_internal_representation
            node1: node representing the start location (id)
            node2: node representing the end location (id)

        Returns:
            a list of node IDs representing the shortest path (in terms of
            distance) from node1 to node2
        """
        # min_f = {k:float("infinity") for k in map_rep}
        print("node1_id", start)
        print("node2_id", end)
        inf = float("infinity")
        min_f = {}
        min_f[start] = 0
        #Parent hash map matching nodes to their parent IDs
        parent = {}
        #Create set not_visited of node IDs
        visited = set()
        #open list of nodes
        open = { start }
        prev_id = start
        #i is id of the adjacent node. NODES is a hash map of nodes with key values as id
        while(len(visited)<self.height*self.width):
            source_id = self.next_id(map_rep, min_f, open, prev_id, end)
            #print("source_id", source_id)
            if (not source_id): #If next_id returns None
                #print("no source")
                break
            if (source_id == end):
                #print("reached node 2")
                visited.add(end)
                break
            adj_nodes = self.generate_adj_nodes(map_rep, source_id)
            for adj in adj_nodes:
                if (adj not in visited):
                    #Check if there isn't yet a minimum distance for the adjacent vertex
                    if (min_f.get(adj, inf) > min_f[source_id]+1):
                        #Updated minimum distance
                        #g always 1 because pixel next to it always 1?
                        min_f[adj] = min_f[source_id]+1
                        parent[adj] = source_id
                    open.add(adj)
            #Mark source as visited
            visited.add(source_id)
            open.remove(source_id)
            prev_id = source_id
        path = []
        if (end not in visited):
            return None
        current_node_id = end
        while(current_node_id!=start):
            path.append(self.get_point(current_node_id))
            current_node_id = parent[current_node_id]
        #path.append(self.get_point(current_node_id))
        return path[::-1]


    def publish_path(self):
        ## CODE FOR PATH PLANNING ##
        while(not self.map_resolved or not self.start_resolved or not self.end_resolved):
            pass
        self.time = rospy.get_time()
        print("self.start_point", self.start_point)
        print("Start A star")
        self.traj = self.a_star(self.map, (int(self.start_point[0]),int(self.start_point[1])), (int(self.end_point[0]),int(self.end_point[1])))
        print("Done w A Star")
        for p in self.traj:
            self.trajectory.addPoint(GridSquare(p[0], p[1]))
        #TODO: Trajectory in real life coords
        # publish trajectory
        if (self.ERROR == 1):
             self.log_distances.log(str(rospy.get_time()-self.time),[self.trajectory.update_distances()])
             traj_log = "/home/racecar/trajectory_AStar_1-1_log_" + str(rospy.get_time())+ ".csv"
             self.trajectory.save(traj_log)
             #1-2: trajectory 1, Log 2
        self.traj_pub.publish(self.trajectory.toPoseArray())
        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()