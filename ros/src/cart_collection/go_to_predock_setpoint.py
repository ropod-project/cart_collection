import os
import smach
import rospy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

class GoToPreDockSetpoint(smach.State):
    def __init__(self, timeout=60.0,
                 nav_goal_topic_name='/route_navigation/goal',
                 nav_feedback_topic_name='/route_navigation/feedback',
                 localisation_topic='/amcl_pose'):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['pre_dock_setpoint'])
        self.nav_goal_pub = rospy.Publisher(nav_goal_topic_name,
                                            ManeuverNavGoal,
                                            queue_size=1)
        self.nav_feedback_sub = rospy.Subscriber(nav_feedback_topic_name,
                                                 ManeuverNavFeedback,
                                                 self.feedback_callback)
        self.robot_pose_sub = rospy.Subscriber(localisation_topic,
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None
        self.robot_pose = None

    def execute(self, userdata):
        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for GoToPreDockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'

        # reconfigure to fine grained navigaiton
        #os.system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_x 0.3 &");
        #os.system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_theta 0.8 &");

        os.system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS max_vel_y 0.5 &")
        os.system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_nh 0 &")
        os.system("rosrun dynamic_reconfigure dynparam set /maneuver_navigation/TebLocalPlannerROS weight_kinematics_forward_drive 0 &")

        # Send goal
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = True
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.pre_dock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout and self.feedback is None:
            rospy.sleep(0.1)

        if self.feedback is not None:
            if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                return 'reached_setpoint'
            if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                return 'setpoint_unreachable'
        return 'timeout'

    def feedback_callback(self, msg):
        self.feedback = msg

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
