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

        print("# of found objects = " len(res.objects))    
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
        self.timeout = timeout

    def execute(self, userdata):
        return 'setpoint_unreachable'


class GoToPreDockSetpoint(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['reached_setpoint',
                                       'setpoint_unreachable',
                                       'timeout'],
                             input_keys=['pre_dock_setpoint'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'setpoint_unreachable'



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


        '
