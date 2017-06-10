import copy
import rospy
import tf.transformations
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseStamped


class State:
    def __init__(self, x=0.0, y=0.0, theta=0.0, parent=None):
        self.x = x
        self.y = y
        self.theta = theta
        self.parent = parent

    @staticmethod
    def from_pose(pose):
        new_state = State()
        new_state.x = pose.position.x
        new_state.y = pose.position.y

        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quaternion)
        new_state.theta = yaw
        return new_state


    def dist_to(self, o_s):
        return ((self.x - o_s.x)**2 + (self.y - o_s.y)**2)**0.5

    def apply(self, move):
        return State(self.x + move.dx, self.y + move.dy, self.theta + move.dtheta)

    def try_apply(self, _map, move, robot):
        model_state = copy.copy(self)
        model_state.parent = self

        goal = self.apply(move)
        steps_count = max(int(self.dist_to(goal) / min(robot.height, robot.width)), 1)
        step = 1.0 / steps_count

        for i in range(steps_count):
            model_state.x += move.dx * step
            model_state.y += move.dy * step
            model_state.theta += move.dtheta * step

            if not _map.is_allowed(model_state, robot):
                return None
        return goal

    def to_pose_stamped(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.25
        # TODO angle a little bit later
        return pose







