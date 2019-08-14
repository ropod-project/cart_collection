import smach
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal

from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose, is_pose_in_polygon

class GetSetpointInPreDockArea(smach.State):
    '''
    Sets a target pose for the robot in front of the cart at the specified distance.
    It also checks that the pose is within the docking area (not just the sub area).

    This is the pre dock pose, and is assumed to be within a reasonable distance of
    the cart, such that the cart would be visible by rear-facing sensors on the robot
    for final approach.
    '''
    def __init__(self, timeout=5.0,
                 robot_length_m=0.73,
                 cart_length_m=0.81,
                 distance_to_cart_m=1.):
        smach.State.__init__(self,
                             outcomes=['setpoint_found', 'setpoint_unreachable', 'timeout'],
                             input_keys=['cart_area', 'cart_pose'],
                             output_keys=['pre_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.robot_length_m = robot_length_m
        self.cart_length_m = cart_length_m
        self.distance_to_cart_m = distance_to_cart_m
        self.cart_predock_pub = rospy.Publisher("cart_predock_pose",
                                                PoseStamped,
                                                queue_size=1)
        self.get_shape_client = actionlib.SimpleActionClient("get_shape", GetShapeAction)

    def execute(self, userdata):
        userdata.pre_dock_setpoint = None

        cart_pose = userdata.cart_pose

        # NOTE: Both poses are in the center of ropod/cart
        distance = (self.robot_length_m + self.cart_length_m) / 2. + self.distance_to_cart_m
        pre_dock_setpoint = get_setpoint_in_front_of_pose(cart_pose, distance)

        if pre_dock_setpoint is None:
            return 'setpoint_unreachable'

        shape_action_server_available = self.get_shape_client.wait_for_server(timeout=self.timeout)
        if not shape_action_server_available:
            rospy.logerr("[cart_collector] Timed out waiting for get_shape action server")
            return 'timeout'

        goal = GetShapeGoal()
        goal.id = int(userdata.cart_area)
        goal.type = 'area'
        self.get_shape_client.send_goal(goal)
        shape_success = self.get_shape_client.wait_for_result(timeout=self.timeout)
        if not shape_success:
            rospy.logerr("[cart_collector] Timed out waiting for shape of docking area")
            return 'timeout'
        shape_result = self.get_shape_client.get_result()

        # TODO: need to check if the pose is reachable too
        # for example, the pose could be within the area but too close to a wall
        #if (not is_pose_in_polygon(pre_dock_setpoint, shape_result.shape.vertices)):
        #    rospy.logerr("[cart_collector] pre dock setpoint is outside docking area")
        #    return 'setpoint_unreachable'

        userdata.pre_dock_setpoint = pre_dock_setpoint
        rospy.loginfo("[cart_collector] pre_dock_setpoint = " + str(pre_dock_setpoint))
        self.cart_predock_pub.publish(pre_dock_setpoint)
        return 'setpoint_found'
