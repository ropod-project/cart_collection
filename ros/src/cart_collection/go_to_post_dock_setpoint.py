import smach
import rospy

class GoToPostDockSetpoint(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['post_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)

    def execute(self, userdata):
        return 'setpoint_unreachable'
