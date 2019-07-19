import smach
import rospy

from geometry_msgs.msg import PoseStamped

from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose

class GetSetpointInPreDockArea(smach.State):
    def __init__(self, timeout=5.0,
                 robot_length_m=0.73,
                 cart_length_m=0.81,
                 distance_to_cart_m=1.,
                 cart_predock_pose_topic='/cart_collection/cart_predock_pose'):
        smach.State.__init__(self,
                             outcomes=['setpoint_found', 'setpoint_unreachable'],
                             input_keys=['cart_pose'],
                             output_keys=['pre_dock_setpoint'])
        self.cart_predock_pub = rospy.Publisher(cart_predock_pose_topic,
                                                PoseStamped,
                                                queue_size=1)
        self.timeout = timeout
        self.robot_length_m = robot_length_m
        self.cart_length_m = cart_length_m
        self.distance_to_cart_m = distance_to_cart_m

    def execute(self, userdata):
        userdata.pre_dock_setpoint = None

        #cart_pose = geometry_msgs.msg.PoseStamped()
        #cart_pose.header.frame_id = "map"
        #cart_pose.pose.position.x = 42
        #cart_pose.pose.position.y = 43
        #cart_pose.pose.orientation.w = 0.707
        #cart_pose.pose.orientation.x = 0.0
        #cart_pose.pose.orientation.y = 0.0
        #cart_pose.pose.orientation.z = -0.707

        cart_pose = userdata.cart_pose

        # NOTE: Both poses are in the center of ropod/cart
        distance = (self.robot_length_m + self.cart_length_m) / 2. + self.distance_to_cart_m
        pre_dock_setpoint = get_setpoint_in_front_of_pose(cart_pose, distance)

        if pre_dock_setpoint is not None:
            userdata.pre_dock_setpoint = pre_dock_setpoint
            rospy.loginfo("[cart_collector] pre_dock_setpoint = " + str(pre_dock_setpoint))
            self.cart_predock_pub.publish(pre_dock_setpoint)
            return 'setpoint_found'
        return 'setpoint_unreachable'
