from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

import rospy
import numpy as np
import tf.transformations

class VisualizationTools:
    @staticmethod
    def plot_line(x, y, publisher, color = (1., 0., 0.), frame = "/base_link"):
        """
        Publishes the points (x, y) to publisher
        so they can be visualized in rviz as
        connected line segments.
        Args:
            x, y: The x and y values. These arrays
            must be of the same length.
            publisher: the publisher to publish to. The
            publisher must be of type Marker from the
            visualization_msgs.msg class.
            color: the RGB color of the plot.
            frame: the transformation frame to plot in.
        """
        # Construct a line
        line_strip = Marker()
        line_strip.type = Marker.LINE_STRIP
        line_strip.header.frame_id = frame

        # Set the size and color
        line_strip.scale.x = 0.1
        line_strip.scale.y = 0.1
        line_strip.color.a = 1.
        line_strip.color.r = color[0]
        line_strip.color.g = color[1]
        line_strip.color.g = color[2]

        # Fill the line with the desired values
        for xi, yi in zip(x, y):
            p = Point()
            p.x = xi
            p.y = yi
            line_strip.points.append(p)

        # Publish the line
        publisher.publish(line_strip)
    
    @staticmethod
    def plot_tree(tree, publisher, color=(1., 0., 0.), frame = "/base_link"):
        # Publish all the particles
        msg = PoseArray()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"
        # msg.child_frame_id = ""
        msg.poses = []
        for node in tree.keys():
            parent = tree[node][0]

            if parent == None:
                continue


            pose = Pose()

            pose.position.x = node[0]
            pose.position.y = node[1]

            #angle to parent
            dx = parent[0] - node[0]
            dy = parent[1] - node[1]
            angle = np.arctan2(dy, dx)

            quat = tf.transformations.quaternion_from_euler(0, 0, angle)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]

            msg.poses.append( pose )
        publisher.publish(msg)