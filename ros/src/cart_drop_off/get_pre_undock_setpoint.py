import smach
import rospy

from geometry_msgs.msg import PoseStamped

from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose

class GetSetpointInPreUndockArea(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['setpoint_found', 'setpoint_unreachable'],
                             output_keys=['pre_undock_setpoint'])
        self.cart_pre_undock_pub = rospy.Publisher("cart_pre_undock_pose",
                                                PoseStamped,
                                                queue_size=1)
        self.timeout = timeout

    def execute(self, userdata):
        userdata.pre_undock_setpoint = None
        return 'setpoint_unreachable'
