import rospy
from smach import StateMachine

from cart_collection.align_and_approach_cart import AlignAndApproachCart
from cart_collection.couple_to_cart import CoupleToCart


class CartRedockSM(StateMachine):
    '''
    Exposes a state machine for cart redocking.
    This state machine is used in the case where the ropod was previously docked
    to a cart, but has detached unintentionally.
    It assumes that the ropod is with a few 10s of centimeters ahead of the cart,
    more or less aligned with the cart, and simply has to align and approach the cart,
    and redock.

    The state machine expects several parameters to be made available to the ROS parameter server,
    which are required by various docking states:

    offset_to_approach_pose_m: float -- robot-cart offset in meters (default 0.55)
    backward_vel_docking_ms: float -- maximum backward approach velocity in m/s (default 0.1)
    max_rot_vel_docking_rads: float -- maximum rotational approach velocity in rad/s (default 0.1)
    approach_x_thresh_m: float -- cart approach threshold along x in meters (default 0.05)
    approach_y_thresh_m: float -- cart approach threshold along y in meters (default 0.05)
    approach_yaw_thresh_rad: float -- cart approach threshold for the orientation
                                      in rad/s (default 0.1)

    max_coupling_attempts: int -- maximum number of cart coupling attempts before giving up (default 3)


    @author Wouter Houtman, Santosh Thoduka, Sebastian Blumenthal
    @contact w.houtman@tue.nl, santosh.thoduka@h-brs.de, blumenthal@locomotec.com

    '''
    def __init__(self, action_req, action_server):
        StateMachine.__init__(self, outcomes=['done', 'failed'])

        # align and approach cart state params
        offset_to_approach_pose_m = float(rospy.get_param('~redock_offset_to_approach_pose_m', '0.55'))
        backward_vel_docking_ms = float(rospy.get_param('~backward_vel_docking_ms', '0.1'))
        max_rot_vel_docking_rads = float(rospy.get_param('~max_rot_vel_docking_rads', '0.1'))
        approach_x_thresh_m = float(rospy.get_param('~approach_x_thresh_m', '0.05'))
        approach_y_thresh_m = float(rospy.get_param('~approach_y_thresh_m', '0.05'))
        approach_yaw_thresh_rad = float(rospy.get_param('~approach_yaw_thresh_rad', '0.1'))

        # couple to cart state params
        max_coupling_attempts = int(rospy.get_param('max_coupling_attempts', '3'))

        self.userdata.action_req = action_req
        self.userdata.action_server = action_server

        with self:
            StateMachine.add('ALIGN_AND_APPROACH_CART', AlignAndApproachCart(offset_to_approach_pose_m=offset_to_approach_pose_m,
                                                                             backward_vel_docking_ms=backward_vel_docking_ms,
                                                                             max_rot_vel_docking_rads=max_rot_vel_docking_rads,
                                                                             approach_x_thresh_m=approach_x_thresh_m,
                                                                             approach_y_thresh_m=approach_y_thresh_m,
                                                                             approach_yaw_thresh_rad=approach_yaw_thresh_rad),
                             transitions={'approach_succeeded': 'COUPLE_TO_CART',
                                          'cart_not_found': 'failed',
                                          'cart_pose_publisher_not_available': 'failed',
                                          'timeout': 'ALIGN_AND_APPROACH_CART'})

            StateMachine.add('COUPLE_TO_CART', CoupleToCart(max_coupling_attempts=max_coupling_attempts),
                             transitions={'coupling_succeeded': 'done',
                                          'coupling_failed': 'ALIGN_AND_APPROACH_CART',
                                          'cannot_switch_to_load_mode': 'failed'})

