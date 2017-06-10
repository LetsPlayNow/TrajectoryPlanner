import rospy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Path

from move import Move
from state import State
from robot import Robot
from map import Map

class TrajectoryPlanner:
    def __init__(self):
        self.map = None
        self.start = None
        self.goal = None

        self.moves = [Move(0.1, 0, 0),  Move(0, 0.1, 0),
                      Move(-1.0, 0, 0), Move(0, -0.1, 0)]
        self.robot = Robot(0.1, 0.1)
        self.is_working = False # Replace with mutex after all

        self.map_subscriber = rospy.Subscriber("grid_map", OccupancyGrid, self.new_map_callback)
        self.start_subscriber = rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.new_start_callback)
        self.goal_subscriber = rospy.Subscriber("goal", PoseStamped, self.new_goal_callback)

        self.path_publisher = rospy.Publisher("trajectory", Path, queue_size=4)
        self.pose_publisher = rospy.Publisher("debug_pose", PoseStamped, queue_size=4)

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
            self.is_working = False

    # In future we can create field planner, which will contain object of algorhytm
    def replan(self):
        # Now just publish plan
        path = Path()
        path.header.frame_id="map"
        rospy.loginfo("Planning was started...")
        for move in self.moves:
            new_state = self.start.try_apply(self.map, move, self.robot)
            if new_state is not None:
                path.poses.append(new_state.to_pose_stamped())
                rospy.loginfo("New state!")
        self.path_publisher.publish(path)
        rospy.loginfo("Planning was finished...")


def main():
    rospy.init_node("trajectory_planner")
    planner = TrajectoryPlanner()
    rospy.spin()

main()