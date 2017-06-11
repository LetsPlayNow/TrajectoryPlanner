import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path
from visualization_msgs.msg import MarkerArray, Marker
import visualization_msgs
import copy

from move import Move
from state import State
from robot import Robot
from map import Map

class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None

        self.moves = [Move(0.05, 0, 0),  Move(0, 0.05, 0),
                      Move(-0.05, 0, 0), Move(0, -0.05, 0)]
        self.robot = Robot(0.1, 0.1)
        self.is_working = False # Replace with mutex after all

        self.map_subscriber = rospy.Subscriber("grid_map", OccupancyGrid, self.new_map_callback)
        self.start_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.new_start_callback)
        self.goal_subscriber = rospy.Subscriber("goal", PoseStamped, self.new_goal_callback)

        self.path_publisher = rospy.Publisher("trajectory", MarkerArray, queue_size=1)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=1)

    def ready_to_plan(self):
        return self.map is not None and self.start is not None and self.goal is not None

    def new_goal_callback(self, goal_pose):
        if not self.is_working:
            self.is_working = True
            new_goal = State.from_pose(goal_pose.pose)
            if self.map is not None and self.map.is_allowed(new_goal, self.robot):
                self.goal = new_goal
                rospy.loginfo("New goal was set")
                if self.ready_to_plan():
                    self.replan()
            self.is_working = False

    def new_start_callback(self, start_pose):
        if not self.is_working:
            self.is_working = True
            new_start = State.from_pose(start_pose.pose.pose)
            if self.map is not None and self.map.is_allowed(new_start, self.robot):
                self.start = new_start
                rospy.loginfo("New start was set")
                if self.ready_to_plan():
                    self.replan()
            self.is_working = False

    def new_map_callback(self, grid_map):
        if not self.is_working:
            self.is_working = True
            self.map = Map(grid_map)
            rospy.loginfo("New map was set")
            if self.ready_to_plan():
                self.replan()
            self.is_working = False

    # In future we can create field planner, which will contain object of algorhytm
    # A* algorhytm based on http://web.mit.edu/eranki/www/tutorials/search/
    def replan(self):
        rospy.loginfo("Planning was started...")
        final_state = None
        opened = [self.start]
        closed = []

        while opened and final_state is None:  # or while we have enough time
            q = min(opened, key=lambda state: state.f)
            opened.remove(q)
            self.pose_publisher.publish(q.to_pose_stamped())
            for move in self.moves:
                successor = q.try_apply(self.map, move, self.robot)
                if successor is not None:
                    if successor.dist_to(self.goal) < 0.1:
                        final_state = successor
                        break
                    successor.g = q.g + successor.dist_to(q)
                    successor.h = successor.dist_to(self.goal)
                    successor.f = successor.g + successor.h
                    successor.parent = q

                    better_node_with_same_position_exists_in_opened = any(other_successor.is_same_as(successor) and other_successor.f < successor.f for other_successor in opened)
                    better_node_with_same_position_exists_in_closed = any(other_successor.is_same_as(successor) and other_successor.f < successor.f for other_successor in closed)
                    if not(better_node_with_same_position_exists_in_opened or better_node_with_same_position_exists_in_closed):
                        opened.append(successor)
                        # When we new state, which better, we should delete old states from this list (i hope they will stay in references of other objects to reconstruct the path)
                        # Or maybe Because, we don't need these old states path, because we can't get in this state second times with better results, having it's in a previous states
            closed.append(q)

        if final_state is None:
            rospy.loginfo("No path found")
        else:
            # Restore and publish path
            rospy.loginfo("Restoring path from final state...")
            path = self.restore_path(final_state)
            self.path_publisher.publish(path)
            rospy.loginfo("Planning was finished...")

    def replan_width(self):
        rospy.loginfo("Planning was started...")
        final_state = None
        opened = [self.start]
        closed = []

        while opened and final_state is None:  # or while we have enough time
            item = opened.pop()
            self.pose_publisher.publish(item.to_pose_stamped())
            for move in self.moves:
                new_item = item.try_apply(self.map, move, self.robot)
                if new_item is not None:
                    if new_item.dist_to(self.goal) < 0.1:
                        final_state = new_item
                        break
                    else:
                        if not any(other_item.is_same_as(new_item) for other_item in closed):
                            opened.insert(0, new_item)

            closed.append(item)

        if final_state is None:
            rospy.loginfo("No path found")
        else:
            # Restore and publish path
            rospy.loginfo("Restoring path from final state...")
            path = self.restore_path(final_state)
            self.path_publisher.publish(path)
            rospy.loginfo("Planning was finished...")

    def restore_path(self, final_state):
        current_state = copy.copy(final_state)
        path = MarkerArray()
        pose_id = 0
        while True:
            pose_marker = current_state.to_marker(self.robot)
            pose_marker.id = pose_id
            path.markers.append(pose_marker)

            current_state = current_state.parent
            pose_id += 1

            if current_state is None:
                break
        return path


def main():
    rospy.init_node("trajectory_planner")
    planner = TrajectoryPlanner()
    rospy.spin()

main()