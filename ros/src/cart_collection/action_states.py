import smach
import rospy

from ropod_ros_msgs.msg import DockResult

class GetCartCollectionGoal(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['dock', 'undock', 'invalid_goal'],
                             input_keys=['collect_cart_goal'],
                             output_keys=['cart_area', 'cart_pose',
                                          'pre_dock_setpoint', 'post_dock_setpoint'])

    def execute(self, userdata):
        # we clear the state machine memory before processing the goal
        userdata.cart_pose = None
        userdata.pre_dock_setpoint = None
        userdata.post_dock_setpoint = None

        action_type = userdata.collect_cart_goal.action.type
        if action_type == 'DOCK':
            rospy.loginfo('[cart_collector] Received DOCK action request')
            if not userdata.collect_cart_goal.action.areas:
                rospy.logerr('[cart_collector] An area has to be specified for docking; aborting request')
                return 'invalid_goal'
            cart_area = userdata.collect_cart_goal.action.areas[0].id
            rospy.loginfo('[cart_collector] Processing request for docking in area %s', cart_area)
            userdata.cart_area = cart_area
            return 'dock'
        elif action_type == 'UNDOCK':
            rospy.loginfo('[cart_collector] Received UNDOCK action request')
            if not userdata.collect_cart_goal.action.areas:
                rospy.logerr('[cart_collector] An area has to be specified for undocking; aborting request')
                return 'invalid_goal'
            cart_area = userdata.collect_cart_goal.action.areas[0].id
            rospy.loginfo('[cart_collector] Processing request for undocking in area %s', cart_area)
            userdata.cart_area = cart_area
            return 'undock'
        else:
            rospy.logerr('[cart_collector] Unknown action type %s; ignoring request', action_type)
            return 'invalid_goal'

class SetActionResult(smach.State):
    def __init__(self, success):
        smach.State.__init__(self, outcomes=['done'],
                             output_keys=['collect_cart_result'])
        self.success = success

    def execute(self, userdata):
        action_result = DockResult()
        action_result.success = self.success
        userdata.collect_cart_result = action_result
        return 'done'
