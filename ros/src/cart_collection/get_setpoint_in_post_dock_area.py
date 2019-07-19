import smach
import rospy

class GetSetpointInPostDockArea(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self, outcomes=['setpoint_found',
                                             'setpoint_unreachable'],
                             output_keys=['post_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)

    def execute(self, userdata):
        return 'setpoint_unreachable'
