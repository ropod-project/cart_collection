import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose

class GetSetpointInPostDockArea(smach.State):
    '''
    Sets the target pose for the robot to reach after docking has been completed.
    The purpose of this target pose is so that robot+cart are sufficiently far away
    from the wall to perform navigation maneuvers.
    '''
    def __init__(self, timeout=5.0,
                      distance_to_move=0.4):
        smach.State.__init__(self, outcomes=['setpoint_found',
                                             'setpoint_unreachable',
                                             'timeout'],
                             output_keys=['post_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.robot_pose = None
        self.distance_to_move = distance_to_move
        self.cart_postdock_pose_pub = rospy.Publisher("cart_postdock_pose",
                                                PoseStamped,
                                                queue_size=1)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)

    def execute(self, userdata):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GetSetpointInPostDockArea not met: Robot pose not available. Aborting.")
            return 'timeout'

        post_dock_setpoint = get_setpoint_in_front_of_pose(self.robot_pose, self.distance_to_move)

        if post_dock_setpoint is not None:
            userdata.post_dock_setpoint = post_dock_setpoint
            rospy.loginfo("[cart_collector] post_dock_setpoint = " + str(post_dock_setpoint))
            self.cart_postdock_pose_pub.publish(post_dock_setpoint)
            return 'setpoint_found'
        return 'setpoint_unreachable'

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
