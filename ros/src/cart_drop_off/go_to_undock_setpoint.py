import smach
import rospy

from geometry_msgs.msg import Twist

class GoToUndockSetpoint(smach.State):
    '''
    Sends a navigation goal to the undock pose and waits until the goal has been reached.
    '''
    def __init__(self, timeout=15.0,
                 preundock_offset_m=0.5,
                 undock_offset_m=0.2):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                            input_keys=['undock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):
        vel = Twist()
        vel.linear.x = -0.3
        # TODO: make this configurable
        runtime = rospy.Duration.from_sec(0.8 / abs(vel.linear.x))
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= runtime:
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.01)
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        return 'reached_setpoint'

