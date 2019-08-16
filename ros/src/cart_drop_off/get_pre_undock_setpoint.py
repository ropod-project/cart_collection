import smach
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped

from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal

from cart_collection.cart_collection_utils import get_pose_perpendicular_to_wall

class GetSetpointInPreUndockArea(smach.State):
    '''
    Sets the pre undock pose. The pose is set at a a distance from the largest edge of the
    specified sub area and oriented perpendicular to it (inwards into to sub area)

    1. get geometry of sub area (via query to the world model mediator)
    2. calculate pose perpendicular to the largest edge of the sub area
    '''
    def __init__(self, timeout=5.0,
                preundock_offset_m=0.5,
                map_frame_name='map'):
        smach.State.__init__(self,
                             outcomes=['setpoint_found', 'setpoint_unreachable', 'timeout'],
                             input_keys=['cart_area', 'cart_sub_area'],
                             output_keys=['pre_undock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.preundock_offset = preundock_offset_m
        self.map_frame_name = map_frame_name
        self.get_shape_client = actionlib.SimpleActionClient("get_shape", GetShapeAction)
        self.cart_pre_undock_pub = rospy.Publisher("cart_pre_undock_pose",
                                                PoseStamped,
                                                queue_size=1)

    def execute(self, userdata):
        userdata.pre_undock_setpoint = None
        shape_action_server_available = self.get_shape_client.wait_for_server(timeout=self.timeout)
        if not shape_action_server_available:
            return 'timeout'

        goal = GetShapeGoal()
        goal.id = int(userdata.cart_sub_area)
        goal.type = 'local_area'
        self.get_shape_client.send_goal(goal)
        shape_success = self.get_shape_client.wait_for_result(timeout=self.timeout)
        if not shape_success:
            rospy.logerr("[cart_collector] Timed out waiting for shape of undocking sub area")
            return 'timeout'
        sub_area_shape_result = self.get_shape_client.get_result()

        goal = GetShapeGoal()
        goal.id = int(userdata.cart_area)
        goal.type = 'corridor'
        self.get_shape_client.send_goal(goal)
        shape_success = self.get_shape_client.wait_for_result(timeout=self.timeout)
        if not shape_success:
            rospy.logerr("[cart_collector] Timed out waiting for shape of undocking area")
            return 'timeout'
        area_shape_result = self.get_shape_client.get_result()

        # TODO: check that the pose is reachable
        cart_pose = get_pose_perpendicular_to_wall(area_shape_result.shape, sub_area_shape_result.shape, self.preundock_offset)
        cart_pose.header.frame_id = self.map_frame_name
        cart_pose.header.stamp = rospy.Time.now()

        self.cart_pre_undock_pub.publish(cart_pose)
        userdata.pre_undock_setpoint = cart_pose
        return 'setpoint_found'
