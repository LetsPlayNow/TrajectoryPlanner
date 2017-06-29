from nav_msgs.msg import OccupancyGrid
from exceptions import IndexError
from geometry_msgs.msg import Point
import rospy
import math

class Map:
    def __init__(self, grid_map):
        self.map = grid_map
        self.width = grid_map.info.width
        self.height = grid_map.info.height
        self.resolution = grid_map.info.resolution

        self.origin = Point()
        self.origin.x = grid_map.info.origin.position.x
        self.origin.y = grid_map.info.origin.position.y

    def get_by_index(self, i, j):
        if not self.are_indices_in_range(i, j):
            raise IndexError()
        return self.map.data[i*self.width + j]

    # i is for row (y), j is for col (x)
    def get_by_coord(self, x, y):
        return self.get_by_index(*self.coord_to_indices(x, y))

    def coord_to_indices(self, x, y):
        i = int((y - self.origin.y) / self.resolution)
        j = int((x - self.origin.x) / self.resolution)
        return (i, j)

    def are_indices_in_range(self, i, j):
        return 0 <= i < self.height and 0 <= j < self.width

    def is_allowed(self, state, robot):
        was_error = False
        i, j = self.coord_to_indices(state.x, state.y)
        side = int(math.floor((max(robot.width, robot.height) / self.resolution) / 2))
        try:
            for s_i in range(i-side, i+side):
                for s_j in range(j-side, j+side):
                    cell = self.get_by_index(s_i, s_j)
                    if cell == 100 or cell == -1:
                        return False
        except IndexError as e:
            # rospy.loginfo("Indices are out of range")
            was_error = True
        return True and not was_error



