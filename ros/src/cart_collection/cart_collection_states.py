import rospy
import smach
import actionlib
import ropod_ros_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import maneuver_navigation.msg

def get_yaw_from_pose(pose):
        orientation_q = pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion (orientation_list)

        return yaw

def get_setpoint_in_front_of_pose(pose, distance):
    setpoint = None
    if(pose == None):
        rospy.logerr("Preconditon for get_setpoint_in_front_of_pose not met: Pose not found. Aborting.")
        return None

    if(pose.header.frame_id != "map"):
        rospy.logerr("Preconditon for get_setpoint_in_front_of_pose not met: Pose is not in map frame. Aborting.")
        return None

    yaw = get_yaw_from_pose(pose)

    x_cart_in_world_frame = pose.pose.position.x
    y_cart_in_world_frame = pose.pose.position.y
    x = x_cart_in_world_frame + distance * math.cos(yaw)
    y = y_cart_in_world_frame + distance * math.sin(yaw)

    setpoint = geometry_msgs.msg.PoseStamped()
    setpoint.header =  pose.header
    setpoint.pose.position.x = x
    setpoint.pose.position.y = y
    setpoint.pose.position.z = pose.pose.position.z
    setpoint.pose.orientation = pose.pose.orientation

    return  setpoint

class GetCartPose(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['cart_found',
                                       'cart_not_found',
                                       'timeout'],
                             input_keys=['docking_area'],
                             output_keys=['cart_pose'])
        self.get_objects_client = actionlib.SimpleActionClient('/get_objects', ropod_ros_msgs.msg.GetObjectsAction)
        self.cart_pose_pub = rospy.Publisher('/cart_collection/selected_cart_pose', geometry_msgs.msg.PoseStamped, queue_size = 1)
        self.timeout = timeout

    def execute(self, userdata):
        #return 'cart_found'
        action_server_available = self.get_objects_client.wait_for_server(timeout = rospy.Duration(self.timeout))
        if (not action_server_available):
               return 'timeout'

        goal = ropod_ros_msgs.msg.GetObjectsGoal()
        goal.area_id = userdata.docking_area
        goal.type = 'carts'
        self.get_objects_client.send_goal(goal)
        result = self.get_objects_client.wait_for_result(timeout = rospy.Duration(self.timeout))
        if (not result):
            return 'timeout'

        res = self.get_objects_client.get_result()
        if (len(res.objects) == 0):
            return 'cart_not_found'

        print("# of found objects = " + str(len(res.objects)))
        res.objects[0].pose.header.frame_id = 'map'
        res.objects[0].pose.header.stamp = rospy.Time.now()
        self.cart_pose_pub.publish(res.objects[0].pose)
        userdata.cart_pose = res.objects[0].pose # fixme
        return 'cart_found'

class LookForCart(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['cart_found',
                                       'cart_not_found',
                                       'timeout'],
                             input_keys=['docking_area'],
                             output_keys=['cart_pose'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'cart_not_found'


class GetSetpointInPreDockArea(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['setpoint_found',
                                       'setpoint_unreachable'],
                             input_keys=['cart_pose'],
                             output_keys=['pre_dock_setpoint'])
        self.cart_predock_pub = rospy.Publisher('/cart_collection/cart_predock_pose', geometry_msgs.msg.PoseStamped, queue_size = 1)
        self.timeout = timeout

    def execute(self, userdata):
        userdata.pre_dock_setpoint = None
        ropod_length = 0.73         # [m]
        cart_length = 0.81          # [m]
        distance_to_cart = 0.4      # [m]

        #cart_pose = geometry_msgs.msg.PoseStamped()
        #cart_pose.header.frame_id = "map"
        #cart_pose.pose.position.x = 42
        #cart_pose.pose.position.y = 43
        #cart_pose.pose.orientation.w = 0.707
        #cart_pose.pose.orientation.x = 0.0
        #cart_pose.pose.orientation.y = 0.0
        #cart_pose.pose.orientation.z = -0.707

        cart_pose = userdata.cart_pose

        distance = (ropod_length + cart_length)/2.0 + distance_to_cart; # NOTE: Both poses are in the center of ropod/cart
        pre_dock_setpoint = get_setpoint_in_front_of_pose(cart_pose, distance_to_cart)

        if(pre_dock_setpoint != None):
            userdata.pre_dock_setpoint = pre_dock_setpoint
            rospy.loginfo("pre_dock_setpoint = " + str(pre_dock_setpoint))
            self.cart_predock_pub.publish(pre_dock_setpoint)
            return 'setpoint_found'

        return 'setpoint_unreachable'




class GoToPreDockSetpoint(smach.State):
    def __init__(self, timeout=20.0):
        smach.State.__init__(self,
                             outcomes=['reached_setpoint',
                                       'setpoint_unreachable',
                                       'timeout'],
                             input_keys=['pre_dock_setpoint'])
        self.nav_goal_pub = rospy.Publisher('/route_navigation/goal', maneuver_navigation.msg.Goal, queue_size = 1)
        self.nav_feedback_sub = rospy.Subscriber('/route_navigation/feedback', maneuver_navigation.msg.Feedback, self.feedback_callback)
        self.robot_pose_sub = rospy.Subscriber('/amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped, self.ropbot_pose_callback)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None
        self.robot_pose = None

    def execute(self, userdata):
        # Get ropot pose
        self.robot_pose = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout:
            if self.robot_pose == None:
                rospy.sleep(0.1)
            else:
                break

        if self.robot_pose == None:
            rospy.logerr("Precondition for GoToPreDockSetpoint not met: Robot pose not available. Aborting.")
            return 'timeout'

        # Send goal
        nav_goal = maneuver_navigation.msg.Goal()
        nav_goal.conf.precise_goal = True
        nav_goal.conf.use_line_planner = True
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = userdata.pre_dock_setpoint
        self.nav_goal_pub.publish(nav_goal)

        # Wait for result
        self.feedback = None
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= self.timeout:
            if self.feedback != None:
                if self.feedback.status == maneuver_navigation.msg.Feedback.SUCCESS:
                    return 'reached_setpoint'
                if self.feedback.status == maneuver_navigation.msg.Feedback.FAILURE_OBSTACLES:
                    return 'setpoint_unreachable'
            else:
                rospy.sleep(0.1)

        return 'timeout'

    def feedback_callback(self, msg):
        self.feedback = msg

    def ropbot_pose_callback(self, msg):
        if(self.robot_pose == None):
            self.robot_pose = geometry_msgs.msg.PosStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose



class AlignAndApproachCart(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['approach_succeeded',
                                       'cart_not_found',
                                       'timeout'])
        self.cart_pose_feedback_sub = rospy.Subscriber('/wm/docking_approach/cart_front', ropod_ros_msgs.msg.ObjectList, self.cart_front_pose_callback)
        self.cmd_vel_pub = rospy.Publisher('/ropod/cmd_vel', geometry_msgs.msg.Twist, queue_size = 1)
        self.cart_front_pose = None
        self.timeout = timeout
        self.pose_reached =  False

    def execute(self, userdata):
        self.cart_front_pose = None

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time <= self.timeout) and not self.pose_reached:
            if self.cart_front_pose != None:
                (vel, self.pose_reached) = self.calculate_final_approach_velocities(self.cart_front_pose)
                cmd_vel_pub.publish(vel)
            else:
                rospy.logwarn("Precondition for AlignAndApproachCart not met: No cart_front_pose reveived so far. Retrying.")

            rospy.sleep(0.1)

        if self.pose_reached:
            return 'approach_succeeded'

        return 'timeout'

    def cart_front_pose_callback(self, msg):
        if(len(msg.objects) > 0):
            self.cart_front_pose =  msg.objects[0]
        if(len(msg.objects) > 1):
            rospy.logwarn("Precondition for cart_front_pose_callback not met: Only one cart front beeing detected. Taking the first one.")

    def calculate_final_approach_velocities(self, pose):
        # Paremeters
        BACKWARD_VEL_DOCKING = 0.1  # [m/s]
        MAX_ROT_VEL_DOCKING = 0.1   # [rad/s]
        x_thresh = 0.05             # [m/s]
        y_thresh = 0.05             # [m/s]
        yaw_thresh = 0.1            # [rad/s]

        vel = geometry_msgs.msg.Twist()
        vel.linear.x  = 0.0
        vel.linear.y  = 0.0
        vel.angular.z = 0.0
        yaw = get_yaw_from_pose(pose)

        if (abs(pose.pose.position.x) < x_thresh) \
            and (abs(pose.pose.position.y) < y_thresh) \
            and (abs(yaw) < yaw_thresh):
            return vel, True # Reached / Done

        if (abs(pose.pose.position.x) > x_thresh) \
            and (abs(pose.pose.position.x) > abs(pose.pose.position.y)):
                # magnitude of BACKWARD_VEL_DOCKING, sign of pose.pose.position.x
                vel.linear.x = math.copysign(BACKWARD_VEL_DOCKING, pose.pose.position.x);
                if (abs(pose.pose.position.y) > y_thresh):
                    x_time = pose.pose.position.x / vel.linear.x;
                    y_vel = pose.pose.position.y / x_time;
                    vel.linear.y = math.copysign(y_vel, pose.pose.position.y);

        if (abs(pose.pose.position.y) > y_thresh) \
            and (abs(pose.pose.position.y) > abs(pose.pose.position.x)):
                vel.linear.y = math.copysign(BACKWARD_VEL_DOCKING, pose.pose.position.y);
                if (abs(pose.pose.position.x) > x_thresh):
                    y_time = pose.pose.position.y / vel.linear.y;
                    x_vel = pose.pose.position.x / y_time;
                    vel.linear.x = math.copysign(x_vel, pose.pose.position.x);

        if (math.abs(yaw) > 0.1):
            # magnitude of MAX_ROT_VEL_DOCKING, sign of yaw
            vel.angular.z = maht.copysign(MAX_ROT_VEL_DOCKING, yaw);

        return vel, False # Not yet done, thus False


class CoupleToCart(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['coupling_succeeded',
                                       'coupling_failed',
                                       'cannot_switch_to_load_mode'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'cannot_switch_to_load_mode'

class GetSetpointInPostDockArea(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['setpoint_found',
                                       'setpoint_unreachable'],
                             output_keys=['post_dock_setpoint'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'setpoint_unreachable'

class GoToPostDockSetpoint(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['reached_setpoint',
                                       'setpoint_unreachable',
                                       'timeout'],
                             input_keys=['post_dock_setpoint'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'setpoint_unreachable'
