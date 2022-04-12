import rospy
import cv2
import numpy as np

from tf.transformations import quaternion_matrix, euler_from_quaternion
from nav_msgs.msg import OccupancyGrid


class Map:
    resolved = False

    def __init__(self, topic, collision_radius, discrete_length):
        self.map_sub = rospy.Subscriber(topic, OccupancyGrid, self.map_callback)
        self.collision_radius = float(collision_radius)
        self.discrete_length = discrete_length

        # while not self.resolved:
        #     pass
        # print(self.get_pixel([0,0]))
        # print(self.get_pixel(self.get_point([1000,-15])))
        # print(self.get_point(self.get_pixel([1000,-15])))

    def show_map(self):
        cv2.imshow("map", self.map)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    
    def map_callback(self, msg):
        if self.resolved:
            # we've already done this, no need to repeat
            return

        self.resolution = float(msg.info.resolution)
        self.discrete_pixels = self.discrete_length/self.resolution
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        # print('width', self.width)
        # print('height', self.height)
        # Scale down to 100 to place on a scale from 0-1
        data = np.array(msg.data, np.double)/100.
        # replace any undetermined pixels (negative values) with 1 to indicate occupancy
        data = np.where(data < 0, 1, data)
        # make binary
        data = np.where(data > 0.5, np.ones(len(data)), np.zeros(len(data)))
        
        self.map = np.reshape(data, (self.height, self.width))

        # position of the map_grid origin wrt to the /map frame
        origin_p = msg.info.origin.position

        # map_grid is rotated around map_grid origin, AFTER it is translated
        origin_o = msg.info.origin.orientation
        origin_o = euler_from_quaternion((origin_o.x, origin_o.y,
                                         origin_o.z, origin_o.w))
        
        # the real world pose of the origin of the map [m, m, rad]
        self.origin = np.array((origin_p.x, origin_p.y))
        self.rotation = origin_o[2]
        

        # safe a dilated copy of the map, for collision detection
        r = int(2.* self.collision_radius / self.resolution)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (r, r))
        self.map_dilated = cv2.dilate(self.map, kernel)
        

        # print(np.shape(self.map))
        # print(np.shape(self.map_dilated))

        rospy.loginfo("New Map Initialized")
        self.resolved = True

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

    def get_block_coords(self, point):
        pixel = self.get_pixel(point)
        
        return (pixel/self.discrete_pixels).astype(int)
        
    def get_adjacent_blocks(self, block):
        blocks = []
        for i in [-1, 0, 1]:
            for j in [-1, 0, 1]:
                new_block = block + np.array([i, j])

                if new_block[0] < 0 or new_block[0] >= int(self.width/self.discrete_pixels):
                    continue
                if new_block[1] < 0 or new_block[1] >= int(self.height/self.discrete_pixels):
                    continue
                blocks.append( new_block )
        
        return blocks

    def is_path_blocked(self, path):
        for point in path:
            if self.is_collision(point):
                return True
        return False

    def is_collision(self, pose):
        pixel = self.get_pixel(pose[:2])
        if pixel[0] < 0 or pixel[0] >= self.width:
            return True
        if pixel[1] < 0 or pixel[1] >= self.height:
            return True
        # print(self.map_dilated[pixel[0], pixel[1]])
        # print(pixel)
        # quit()
        return self.map_dilated[pixel[1], pixel[0]] == 1.0

    def random_point(self):
        """
            sample randomly from the real world space. 
            returns (x, y, th) in (m, m, rad)
        """

        #TODO: make sure this distribution actually covers the map nicely
        x,y = self.get_point( np.random.uniform(size=2) * [self.width-1, self.height-1] )
        th = np.random.uniform()*2*np.pi
        return np.array((x, y, th))

    def random_point_clear(self):
        """
        Returns a random point on the map that is not in collision
        """
        # TODO: there is a significantly smarter way to do this given foreknowledge of the map

        rand = self.random_point()
        # Make sure selected point is not in collision
        while self.is_collision(rand):
            rand = self.random_point()
        
        return rand