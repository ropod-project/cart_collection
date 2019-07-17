import rospy
import smach
import actionlib
import ropod_ros_msgs.msg


class GetCartPose(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['cart_found',
                                       'cart_not_found',
                                       'timeout'],
                             input_keys=['docking_area'],
                             output_keys=['cart_pose'])
        self.get_objects_client = actionlib.SimpleActionClient('/get_objects', ropod_ros_msgs.msg.GetObjectsAction)
        self.timeout = timeout

    def execute(self, userdata):
        action_server_available = self.get_objects_client.wait_for_server(timeout = rospy.Duration(5.0))
        if (not action_server_available):
            return 'timeout'
        goal = ropod_ros_msgs.msg.GetObjectsGoal()
        goal.area_id = userdata.docking_area
        goal.type = 'carts'
        self.get_objects_client.send_goal(goal)
        result = self.get_objects_client.wait_for_result(timeout = rospy.Duration(timeout))
        if (not result):
            return 'timeout'

        res = self.get_objects_client.get_result()
        if (len(res.objects) == 0):
            return 'cart_not_found'

        userdata.cart_pose = res.objects[0].pose
        return 'cart_found'
