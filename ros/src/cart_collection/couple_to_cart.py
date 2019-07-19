import smach
import rospy

from std_msgs.msg import Bool
from ropod_ros_msgs.msg import DockingCommand, DockingFeedback

class CoupleToCart(smach.State):
    def __init__(self, timeout=60.0,
                 docking_cmd_topic='/ropod/ropod_low_level_control/cmd_dock',
                 docking_feedback_topic='/ropod/ropod_low_level_control/dockingFeedback',
                 nav_load_attached_topic='/route_navigation/set_load_attached',
                 max_coupling_attempts=3):
        smach.State.__init__(self, outcomes=['coupling_succeeded',
                                             'coupling_failed',
                                             'cannot_switch_to_load_mode'])
        self.docking_cmd_pub = rospy.Publisher(docking_cmd_topic,
                                               DockingCommand,
                                               queue_size=1)
        self.cart_pose_feedback_sub = rospy.Subscriber(docking_feedback_topic,
                                                       DockingFeedback,
                                                       self.docking_feedback_callback)
        self.nav_load_attached_pub = rospy.Publisher(nav_load_attached_topic,
                                                     Bool, queue_size=1)
        self.docking_feedback = None
        self.coupling_attempts = 0
        self.max_coupling_attempts = max_coupling_attempts
        self.timeout = rospy.Duration.from_sec(timeout)

    def execute(self, userdata):
        self.docking_feedback = None
        self.coupling_attempts = 0

        docking_msg = DockingCommand()
        docking_msg.docking_command = DockingCommand.DOCKING_COMMAND_DOCK
        self.docking_cmd_pub.publish(docking_msg)
        self.coupling_attempts = self.coupling_attempts + 1

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout:
            if self.docking_feedback is not None:
                if self.docking_feedback.docking_status == DockingFeedback.DOCKING_FB_DOCKED:
                    #reconfigure ropod to go into "load mode"
                    rospy.loginfo("[cart_collector] Coupling succeeded. Switching navigation component into load mode.")
                    attached_msg = Bool()
                    attached_msg.data = True
                    self.nav_load_attached_pub.publish(attached_msg)
                    return 'coupling_succeeded'
                if self.docking_feedback.docking_status == DockingFeedback.DOCKING_FB_REST:
                    if self.coupling_attempts >= self.max_coupling_attempts:
                        docking_msg.docking_command = DockingCommand.DOCKING_COMMAND_RELEASE
                        self.docking_cmd_pub.publish(docking_msg)
                        return 'coupling_failed'
                    rospy.logwarn("[cart_collector] Coupling attempt " + str(self.coupling_attempts) + " out of " + str(self.max_coupling_attempts) + " failed. Retrying.")
                    self.docking_cmd_pub.publish(docking_msg)
                    self.coupling_attempts = self.coupling_attempts + 1
            else:
                rospy.sleep(0.1)

        docking_msg.docking_command = DockingCommand.DOCKING_COMMAND_RELEASE
        self.docking_cmd_pub.publish(docking_msg)
        return 'coupling_failed'

    def docking_feedback_callback(self, msg):
        self.docking_feedback = msg
