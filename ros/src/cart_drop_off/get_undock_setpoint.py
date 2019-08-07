import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose

class GetUndockSetpoint(smach.State):
    def __init__(self, timeout=5.0,
                 preundock_offset_m=0.5,
                 undock_offset_m=0.2):
        smach.State.__init__(self, outcomes=['setpoint_found',
                                             'setpoint_unreachable',
                                             'timeout'],
                            output_keys=['undock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.robot_pose = None
        # offset from current position = offset from wall (undock) - offset from wall (preundock)
        self.offset_from_current_position = undock_offset_m - preundock_offset_m
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.cart_undock_pose_pub = rospy.Publisher("cart_undock_pose",
                                                PoseStamped,
                                                queue_size=1)

    def execute(self, userdata):
        # Get robot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GetUndockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'
        undock_setpoint = get_setpoint_in_front_of_pose(self.robot_pose, self.offset_from_current_position)
        if undock_setpoint is not None:
            userdata.undock_setpoint = undock_setpoint
            self.cart_undock_pose_pub.publish(undock_setpoint)
            return 'setpoint_found'

        return 'setpoint_unreachable'

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
