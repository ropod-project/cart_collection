import smach
import rospy

from geometry_msgs.msg import Twist

class GoToPostDockSetpoint(smach.State):
    '''
    Sends a navigation goal to the post dock pose and waits until is complete.
    '''
    def __init__(self, timeout=5.0):
        smach.State.__init__(self, outcomes=['reached_setpoint',
                                             'setpoint_unreachable',
                                             'timeout'],
                             input_keys=['post_dock_setpoint'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

    def execute(self, userdata):
        vel = Twist()
        vel.linear.x = 0.1
        # TODO: make this configurable and use local obstacle avoidance
        runtime = rospy.Duration.from_sec(0.3 / vel.linear.x)
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= runtime:
            self.cmd_vel_pub.publish(vel)
            rospy.sleep(0.1)
        zero_vel = Twist()
        self.cmd_vel_pub.publish(zero_vel)
        return 'reached_setpoint'

