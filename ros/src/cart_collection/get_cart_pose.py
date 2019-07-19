import smach
import rospy
import actionlib

from geometry_msgs.msg import PoseStamped
from ropod_ros_msgs.msg import GetObjectsAction, GetObjectsGoal

class GetCartPose(smach.State):
    def __init__(self, timeout=5.0,
                 map_frame_name='map',
                 get_objects_server_name='/get_objects',
                 cart_pose_topic_name='/cart_collection/selected_cart_pose'):
        smach.State.__init__(self, outcomes=['cart_found', 'cart_not_found', 'timeout'],
                             input_keys=['docking_area'],
                             output_keys=['cart_pose'])
        self.get_objects_client = actionlib.SimpleActionClient(get_objects_server_name, GetObjectsAction)
        self.cart_pose_pub = rospy.Publisher(cart_pose_topic_name, PoseStamped, queue_size=1)
        self.timeout = timeout
        self.map_frame_name = map_frame_name

    def execute(self, userdata):
        action_server_available = self.get_objects_client.wait_for_server(timeout=rospy.Duration(self.timeout))
        if not action_server_available:
            return 'timeout'

        goal = GetObjectsGoal()
        goal.area_id = userdata.docking_area
        goal.type = 'carts'
        self.get_objects_client.send_goal(goal)
        result = self.get_objects_client.wait_for_result(timeout=rospy.Duration(self.timeout))
        if not result:
            return 'timeout'

        res = self.get_objects_client.get_result()
        if not res.objects:
            return 'cart_not_found'

        if res.objects[0].pose.header.frame_id != self.map_frame_name:
            rospy.logerr("[cart_collector] Preconditon for GetCartPose not met: Pose is not in map frame. Aborting.")
            return 'cart_not_found'

        rospy.loginfo("[cart_collector] # of found objects = " + str(len(res.objects)))
        self.cart_pose_pub.publish(res.objects[0].pose)
        userdata.cart_pose = res.objects[0].pose # fixme

        return 'cart_found'
