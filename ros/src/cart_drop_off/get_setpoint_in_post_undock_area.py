import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose

class GetSetpointInPostUndockArea(smach.State):
    def __init__(self, timeout=5.0,
                      distance_to_move=0.4):
        smach.State.__init__(self, outcomes=['setpoint_found',
                                             'setpoint_unreachable',
                                             'timeout'],
                             output_keys=['post_undock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.cart_post_undock_pose_pub = rospy.Publisher("cart_post_undock_pose",
                                                PoseStamped,
                                                queue_size=1)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.robot_pose = None
        self.distance_to_move = distance_to_move

    def execute(self, userdata):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GetSetpointInPostUndockArea not met: Robot pose not available. Aborting.")
            return 'timeout'

        post_undock_setpoint = get_setpoint_in_front_of_pose(self.robot_pose, self.distance_to_move)

        if post_undock_setpoint is not None:
            userdata.post_undock_setpoint = post_undock_setpoint
            self.cart_post_undock_pose_pub.publish(post_undock_setpoint)
            return 'setpoint_found'
        return 'setpoint_unreachable'

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
