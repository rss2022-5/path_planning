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

class GridSquare:

    def __init__(self, x, y):
        #Stores pixel coordinates of grid square
        self.x = x
        self.y = y

    def __eq__(self, obj):
        return isinstance(obj, GridSquare) and self.x==obj.x and self.y==obj.y
    
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
        self.odom_topic = rospy.get_param("~odom_topic")
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_cb)
        self.trajectory = LineTrajectory("/planned_trajectory")
        self.goal_sub = rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb, queue_size=10)
        self.traj_pub = rospy.Publisher("/trajectory/current", PoseArray, queue_size=10)
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)

    def map_cb (self, msg):
        #Prepare map representation
        map = np.array(msg.data, np.double)
        map = np.where(map < 0, 100, map)
        map = map/100.0
        map = np.clip(map, 0, 1)
        map = np.where(map > 0.5, 1, 0)
        #TODO: Check kernel size
        kernel = np.ones((5,5),np.uint8)
        map = cv.erode(map, kernel, iterations = 1)
        map = cv.dilate(map, kernel, iterations = 1)
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
        self.origin = (origin_p.x, origin_p.y, origin_o[2])
        self.map_resolved = True

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.start_point = self.real2pixel((x, y, th))
        self.start_resolved = True

    def goal_cb(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        q = msg.pose.orientation
        # THIS MIGHT NEED TO CHANGE TO X INSTEAD OF Y
        _, th, _ = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.end_point = self.real2pixel((x, y, th))
        self.end_resolved = True

    def real2pixel(self, point):
        x, y, th = point
        # multiply (x, y) * self.resolution
        # apply rotation and translation of self.origin.orientation and self.origin.position
        ang = self.origin[2]
        
        rot = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
        #y = -y
        rot = np.append(rot, np.array([[self.origin[0], self.origin[1]]]).T, 1)
        print("append arr", np.array([[self.origin[0], self.origin[1]]]).T)
        rot = np.append(rot, np.array([[0, 0, 1]]), 0)
        print("rot", rot)

        p_matrix = np.array([[x, y, 1]]).T
        rotated = np.dot(rot, p_matrix)
        rotated = rotated/self.resolution
        return (np.rint(rotated[0]).astype(int), np.rint(rotated[1]).astype(int))
    
    def pixel2real(self, point):
        pass

    def get_map (self, msg):
        map = np.reshape(msg.data, (self.height, self.width))
        for i in map:
            for j in map[i]:
                if (map[i][j]>0.5):
                    map[i][j]=1
                else:
                    map[i][j]=0
        kernel = np.ones((5,5),np.uint8)
        map = cv.erode(map, kernel, iterations = 1)
        map = cv.dilate(map, kernel, iterations = 1)
        return map

    def g(map_rep, node1, node2):
        """
        Returns distance between two nodes
        """
        return 1

    def heuristic(map_rep, node1, node2):
        """
        Returns the heuristic for a node
        """
        return 0
    
    def generate_adj_nodes(map_rep, source):
        #TODO Modify
        adj = []
        try:
            g = GridSquare(source.x-1, source.y)
            if (not g.occupied(map_rep)):
                adj.append(g)
        except: pass
        try:
            g = GridSquare(source.x+1, source.y)
            if (not g.occupied(map_rep)):
                adj.append(g)
        except: pass
        try:
            g = GridSquare(source.x, source.y-1)
            if (not g.occupied(map_rep)):
                adj.append(g)
        except: pass
        try:
            g = GridSquare(source.x, source.y+1)
            if (not g.occupied(map_rep)):
                adj.append(g)
        except: pass
        return adj
    
    def next_id(self, map_rep, min_f, open, node2):
        """
        Return the node with the next shortest f (distance + heuristic)
        """
        min = float("infinity")
        id = None
        for k in open:
            if (min_f[k]+self.heuristic(map_rep,node2)<min):
                id = k
                min = min_f[k]+self.heuristic(map_rep,node2)
        return id
        
    def a_star(self, map_rep, node1, node2):
        """
        Return the shortest path between the two nodes

        Parameters:
            map_rep: the result of calling build_internal_representation
            node1: node representing the start location
            node2: node representing the end location

        Returns:
            a list of node IDs representing the shortest path (in terms of
            distance) from node1 to node2
        """
        # min_f = {k:float("infinity") for k in map_rep}
        inf = float("infinity")
        min_f = {}
        min_f[node1] = 0
        #Parent hash map matching nodes to their parent IDs
        parent = {}
        #Create set not_visited of node IDs
        visited = set()
        #open list of nodes
        open = { node1 }
        #i is id of the adjacent node. NODES is a hash map of nodes with key values as id
        while(len(visited)<len(map_rep)):
            source = self.next_id(map_rep, min_f, open, node2)
            if (not source): #If next_id returns None
                break
            if (source == node2):
                visited.add(node2)
                break
            adj_nodes = self.generate_adj_nodes(map_rep, source)
            for adj in adj_nodes:
                if (adj not in visited):
                    #Check if there isn't yet a minimum distance for the adjacent vertex
                    if (min_f.get(adj, inf) > min_f[source]+self.heuristic(map_rep, adj)):
                        #Updated minimum distance
                        #g always 1 because pixel next to it always 1?
                        min_f[adj] = min_f[source]+self.heuristic(map_rep, adj)
                        parent[adj] = source
                    open.add(adj)
            #Mark source as visited
            visited.add(source)
            open.remove(source)
        #Backtrack to get path using parents list
        path = []
        if (node2 not in visited):
            return None
        current_node = node2
        while(current_node!=node1):
            path.append(current_node)
            current_node = parent[current_node]
        path.append(current_node)
        return path[::-1]


    def publish_path(self):
        ## CODE FOR PATH PLANNING ##
        while (self.end_resolved is not True or self.start_resolved is not True or self.map_resolved is not True):
            pass
        start = GridSquare(self.start_point[0], self.start_point[1])
        end = GridSquare(self.end_point[0], self.end_point[1])
        self.traj = self.a_star(map, self.start_point, self.end_point)
        self.trajectory = LineTrajectory()
        for gridsquare in self.traj:
            self.trajectory.addPoint(gridsquare)
        #TODO: Trajectory in real life coords
        # publish trajectory
        self.traj_pub.publish(self.trajectory.toPoseArray())
        # visualize trajectory Markers
        self.trajectory.publish_viz()


if __name__=="__main__":
    rospy.init_node("path_planning")
    pf = PathPlan()
    rospy.spin()