import smach
import rospy

from std_msgs.msg import Bool
from ropod_ros_msgs.msg import DockingCommand, DockingFeedback

class UncoupleFromCart(smach.State):
    '''
    Uncouples the robot to the cart using the docking mechanism and sets 'load_attached' to false
    '''
    def __init__(self, timeout=60.0,
                 nav_load_attached_topic='/route_navigation/set_load_attached',
                 max_uncoupling_attempts=3):
        smach.State.__init__(self, outcomes=['uncoupling_succeeded',
                                             'uncoupling_failed',
                                             'cannot_switch_to_robot_mode'])
        self.docking_cmd_pub = rospy.Publisher("docking_cmd",
                                               DockingCommand,
                                               queue_size=1)
        self.cart_pose_feedback_sub = rospy.Subscriber("docking_feedback",
                                                       DockingFeedback,
                                                       self.docking_feedback_callback)
        self.nav_load_attached_pub = rospy.Publisher("set_load_attached",
                                                     Bool, queue_size=1)
        self.toggle_cart_publisher_client = rospy.ServiceProxy('toggle_cart_publisher_srv', ToggleObjectPublisher)
        self.docking_feedback = None
        self.uncoupling_attempts = 0
        self.max_uncoupling_attempts = max_uncoupling_attempts
        self.timeout = rospy.Duration.from_sec(timeout)

    def execute(self, userdata):
        self.docking_feedback = None
        self.uncoupling_attempts = 0

        docking_msg = DockingCommand()
        docking_msg.docking_command = DockingCommand.DOCKING_COMMAND_RELEASE
        self.docking_cmd_pub.publish(docking_msg)
        self.uncoupling_attempts = self.uncoupling_attempts + 1

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout:
            if self.docking_feedback is not None:
                if self.docking_feedback.docking_status == DockingFeedback.DOCKING_FB_REST:
                    #reconfigure ropod to go into "robot mode"
                    rospy.loginfo("[cart_collector] Uncoupling succeeded. Switching navigation component into robot mode.")
                    attached_msg = Bool()
                    attached_msg.data = False
                    self.nav_load_attached_pub.publish(attached_msg)
                    resp = self.toggle_cart_publisher_client(enable_publisher=False)
                    return 'uncoupling_succeeded'
            else:
                rospy.sleep(0.1)

        return 'uncoupling_failed'

    def docking_feedback_callback(self, msg):
        self.docking_feedback = msg
