import smach
import rospy

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from cart_collection.cart_collection_utils import set_dynamic_navigation_params

class GoToPreDockSetpoint(smach.State):
    '''
    Sends a navigation goal to the pre dock pose and waits until it is complete.
    '''
    def __init__(self, timeout=60.0):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['pre_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None
        self.robot_pose = None
        self.nav_goal_pub = rospy.Publisher("nav_goal",
                                            ManeuverNavGoal,
                                            queue_size=1)
        self.nav_cancel_pub = rospy.Publisher("nav_cancel",
                                              Bool,
                                              queue_size=1)
        self.nav_feedback_sub = rospy.Subscriber("nav_feedback",
                                                 ManeuverNavFeedback,
                                                 self.feedback_callback)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.reconfigure_controller_pub = rospy.Publisher("reconfigure_controller",
                                                          String,
                                                          queue_size=1)

    def execute(self, userdata):
        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GoToPreDockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'

        # reconfigure to fine grained navigation and platform control
        self.set_nav_and_platform_mode('fine_grained')

        # Send goal
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = False
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.pre_dock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout:
            rospy.sleep(0.1)
            if self.feedback is not None:
                if self.feedback.status == ManeuverNavFeedback.BUSY:
                    continue
                else:
                    break

        if self.feedback is not None:
            if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                self.set_nav_and_platform_mode('default')
                return 'reached_setpoint'
            if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                self.set_nav_and_platform_mode('default')
                return 'setpoint_unreachable'
            else:
                print("maneuver feedback ", self.feedback.status)
        self.set_nav_and_platform_mode('default')
        self.nav_cancel_pub.publish(True)
        return 'timeout'

    def feedback_callback(self, msg):
        self.feedback = msg

    def set_nav_and_platform_mode(self, mode):
        if mode == 'default':
            self.reconfigure_controller_pub.publish(String(data='default'))
            set_dynamic_navigation_params('non_holonomic_mode')
        elif mode == 'fine_grained':
            self.reconfigure_controller_pub.publish(String(data='fine_grained'))
            set_dynamic_navigation_params('omni_drive_mode')


    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
