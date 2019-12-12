import smach
import rospy
import std_msgs.msg

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from cart_collection.cart_collection_utils import set_dynamic_navigation_params, is_pose_in_polygon

class GoToPreUndockSetpoint(smach.State):
    '''
    Sends a navigation goal to the pre undock pose and waits until the goal has been reached.
    '''
    def __init__(self, timeout=120.0):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['pre_undock_setpoint', 'cart_sub_area', 'area_shape', 'sub_area_shape'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None
        self.robot_pose = None
        self.nav_goal_pub = rospy.Publisher("nav_goal",
                                            ManeuverNavGoal,
                                            queue_size=1)
        self.nav_feedback_sub = rospy.Subscriber("nav_feedback",
                                                 ManeuverNavFeedback,
                                                 self.feedback_callback)
        self.nav_cancel_pub = rospy.Publisher("nav_cancel",
                                            std_msgs.msg.Bool,
                                            queue_size=1)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)

    def execute(self, userdata):
        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GoToPreUndockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'


        set_dynamic_navigation_params('undocking_speed')
        # Send goal
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = False
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.pre_undock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        robot_inside_sub_area = False
        while rospy.Time.now() - start_time <= self.timeout:
            rospy.sleep(0.1)
            # only check if robot is inside subarea
            if is_pose_in_polygon(self.robot_pose, userdata.sub_area_shape.vertices):
                rospy.loginfo("Robot is within undock subarea; cancelling nav goal")
                robot_inside_sub_area = True
                self.nav_cancel_pub.publish(True)
                break
            if self.feedback is not None:
                if self.feedback.status == ManeuverNavFeedback.BUSY:
                    continue
                else:
                    break

        if robot_inside_sub_area:
            set_dynamic_navigation_params('normal_speed')
            return 'reached_setpoint'

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
