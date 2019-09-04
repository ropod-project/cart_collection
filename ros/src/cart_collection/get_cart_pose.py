import smach
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from ropod_ros_msgs.msg import GetObjectsAction, GetObjectsGoal
from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal
from ropod_ros_msgs.msg import Status

from cart_collection.cart_collection_utils import get_pose_perpendicular_to_edge, send_feedback

class GetCartPose(smach.State):
    '''
    Gets the pose of the cart within the specified sub area using queries to the world model.
    We assume that the cart is always up against a wall / perpendicular to one of the edges
    of the sub area.

    1. Get the pose of the cart by querying the world model mediator
    2. Get the geometry of the sub area
    3. Modify the pose of the cart such that it is roughly perpendicular to one of the
       edges of the sub area
    '''
    def __init__(self, timeout=5.0,
                 map_frame_name='map'):
        smach.State.__init__(self, outcomes=['cart_found', 'cart_not_found', 'timeout'],
                             input_keys=['cart_area', 'cart_sub_area', 'action_req', 'action_server', 'sub_area_shape'],
                             output_keys=['cart_pose', 'action_server', 'area_shape', 'sub_area_shape'])
        self.get_objects_client = actionlib.SimpleActionClient("get_objects", GetObjectsAction)
        self.get_shape_client = actionlib.SimpleActionClient("get_shape", GetShapeAction)
        self.cart_pose_pub = rospy.Publisher("cart_pose", PoseStamped, queue_size=1)
        self.timeout = rospy.Duration.from_sec(timeout)
        self.map_frame_name = map_frame_name

    def execute(self, userdata):
        obj_action_server_available = self.get_objects_client.wait_for_server(timeout=self.timeout)
        shape_action_server_available = self.get_shape_client.wait_for_server(timeout=self.timeout)
        if not obj_action_server_available or not shape_action_server_available:
            return 'timeout'

        # Get shape of docking sub area
        userdata.sub_area_shape = None
        goal = GetShapeGoal()
        goal.id = int(userdata.cart_sub_area)
        goal.type = 'local_area'
        self.get_shape_client.send_goal(goal)
        shape_success = self.get_shape_client.wait_for_result(timeout=self.timeout)
        if not shape_success:
            rospy.logerr("[cart_collector] Timed out waiting for shape of docking area")
            return 'timeout'
        shape_result = self.get_shape_client.get_result()
        userdata.sub_area_shape = shape_result.shape

        # Get shape of docking area
        userdata.area_shape = None
        goal = GetShapeGoal()
        goal.id = int(userdata.cart_area)
        goal.type = 'corridor'
        self.get_shape_client.send_goal(goal)
        shape_success = self.get_shape_client.wait_for_result(timeout=self.timeout)
        if not shape_success:
            rospy.logerr("[cart_collector] Timed out waiting for shape of docking area")
            return 'timeout'
        shape_result = self.get_shape_client.get_result()
        userdata.area_shape = shape_result.shape

        # Get cart entity
        goal = GetObjectsGoal()
        goal.area_id = userdata.cart_sub_area
        goal.type = 'carts'
        self.get_objects_client.send_goal(goal)
        result = self.get_objects_client.wait_for_result(timeout=self.timeout)
        if not result:
            rospy.logerr("[cart_collector] Timed out waiting for cart object")
            return 'timeout'

        res = self.get_objects_client.get_result()
        if not res.objects:
            rospy.logerr('[cart_collector] no cart entities found')
            return 'cart_not_found'

        if res.objects[0].pose.header.frame_id != self.map_frame_name:
            rospy.logerr("[cart_collector] Preconditon for GetCartPose not met: Pose is not in map frame. Aborting.")
            return 'cart_not_found'

        rospy.loginfo("[cart_collector] # of found objects = " + str(len(res.objects)))


        cart_pose = get_pose_perpendicular_to_edge(userdata.sub_area_shape, res.objects[0].pose)
        ## NOTE: we assume there is only one cart in the specified sub area
        self.cart_pose_pub.publish(cart_pose)
        userdata.cart_pose = cart_pose

        send_feedback(userdata.action_req, userdata.action_server, Status.MOBIDIK_DETECTED)

        return 'cart_found'
