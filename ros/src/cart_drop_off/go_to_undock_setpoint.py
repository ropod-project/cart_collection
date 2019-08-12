import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from cart_collection.cart_collection_utils import set_dynamic_navigation_params


class GoToUndockSetpoint(smach.State):
    '''
    Sends a navigation goal to the undock pose and waits until the goal has been reached.
    '''
    def __init__(self, timeout=15.0,
                 preundock_offset_m=0.5,
                 undock_offset_m=0.2):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                            input_keys=['undock_setpoint'])
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
        # Get robot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GoToUndockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'

        set_dynamic_navigation_params('undocking_speed')
        # Send goal
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = True
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.undock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout and self.feedback is None:
            rospy.sleep(0.1)

        if self.feedback is not None:
            if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                set_dynamic_navigation_params('normal_speed')
                return 'reached_setpoint'
            if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                set_dynamic_navigation_params('normal_speed')
                return 'setpoint_unreachable'
        set_dynamic_navigation_params('normal_speed')

        return 'timeout'

    def feedback_callback(self, msg):
        self.feedback = msg

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
