import rospy
import smach
import actionlib
import ropod_ros_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import maneuver_navigation.msg

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

        pre_dock_setpoint = self.get_setpoint_in_front_of_cart(cart_pose, ropod_length, cart_length, distance_to_cart)

        if(pre_dock_setpoint != None):
            userdata.pre_dock_setpoint = pre_dock_setpoint
            rospy.loginfo("pre_dock_setpoint = " + str(pre_dock_setpoint))
            self.cart_predock_pub.publish(pre_dock_setpoint)
            return 'setpoint_found'

        return 'setpoint_unreachable'

    def get_setpoint_in_front_of_cart(self, cart_pose, ropod_length, cart_length, distance_to_cart):
            setpoint = None
            if(cart_pose == None):
                rospy.logerr("Preconditon for getSetpointInFrontOfCart not met: Cart not found. Aborting.")
                return None

            distance = 0.5* ( ropod_length + cart_length ) + distance_to_cart;
            orientation_q = cart_pose.pose.orientation # TODO plausibility check
            orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
            (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
            rospy.loginfo("yaw = " +  str(yaw))
            rospy.loginfo("dist = " + str(distance))

            if(cart_pose.header.frame_id != "map"):
                rospy.logerr("Preconditon for getSetpointInFrontOfCart not met: Cart is not in map frame. Aborting.")
                return None

            x_cart_in_world_frame = cart_pose.pose.position.x
            y_cart_in_world_frame = cart_pose.pose.position.y
            x = x_cart_in_world_frame + distance * math.cos(yaw)
            y = y_cart_in_world_frame + distance * math.sin(yaw)

            setpoint = geometry_msgs.msg.PoseStamped()
            setpoint.header =  cart_pose.header
            setpoint.pose.position.x = x
            setpoint.pose.position.y = y
            setpoint.pose.position.z = cart_pose.pose.position.z
            setpoint.pose.orientation = cart_pose.pose.orientation

            return  setpoint


class GoToPreDockSetpoint(smach.State):
    def __init__(self, timeout=20.0):
        smach.State.__init__(self,
                             outcomes=['reached_setpoint',
                                       'setpoint_unreachable',
                                       'timeout'],
                             input_keys=['pre_dock_setpoint'])
        self.nav_goal_pub = rospy.Publisher('/route_navigation/goal', maneuver_navigation.msg.Goal, queue_size = 1)
        self.nav_feedback_sub = rospy.Subscriber('/route_navigation/feedback', maneuver_navigation.msg.Feedback, self.feedback_callback)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.feedback = None

    def execute(self, userdata):

	nav_goal = maneuver_navigation.msg.Goal()
	nav_goal.conf.precise_goal = True
	nav_goal.conf.use_line_planner = True
	nav_goal.conf.append_new_maneuver = False
	#userdata.pre_dock_setpoint
        self.nav_goal_pub.publish(nav_goal)

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



class AlignAndApproachCart(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['approach_succeeded',
                                       'cart_not_found',
                                       'timeout'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'cart_not_found'

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
