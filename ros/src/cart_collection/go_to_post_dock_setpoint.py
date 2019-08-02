import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from cart_collection.cart_collection_utils import set_omni_drive_mode, reset_to_non_holonomic_mode

class GoToPostDockSetpoint(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['post_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None
        self.robot_pose = None
        self.nav_goal_pub = rospy.Publisher("nav_goal",
                                            ManeuverNavGoal,
                                            queue_size=1)
        self.nav_feedback_sub = rospy.Subscriber("nav_feedback",
                                                 ManeuverNavFeedback,
                                                 self.feedback_callback)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)

    def execute(self, userdata):
        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GoToPostDockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'


        set_omni_drive_mode()
        # Send goal
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = True
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.post_dock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout and self.feedback is None:
            rospy.sleep(0.1)

        if self.feedback is not None:
            if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                reset_to_non_holonomic_mode()
                return 'reached_setpoint'
            if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                reset_to_non_holonomic_mode()
                return 'setpoint_unreachable'
        reset_to_non_holonomic_mode()
        return 'timeout'

    def feedback_callback(self, msg):
        self.feedback = msg

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
