import rospy
from smach import StateMachine

from cart_collection.get_cart_pose import GetCartPose
from cart_collection.look_for_cart import LookForCart
from cart_collection.get_cart_setpoint import GetSetpointInPreDockArea
from cart_collection.go_to_predock_setpoint import GoToPreDockSetpoint
from cart_collection.align_and_approach_cart import AlignAndApproachCart
from cart_collection.couple_to_cart import CoupleToCart
from cart_collection.get_setpoint_in_post_dock_area import GetSetpointInPostDockArea
from cart_collection.go_to_post_dock_setpoint import GoToPostDockSetpoint


class CartCollectionSM(StateMachine):
    '''
    Exposes a state machine for cart docking.

    The state machine expects several parameters to be made available to the ROS parameter server,
    which are required by various docking states:

    map_frame_name: str -- name of the world frame for global localisation (default map)

    robot_length_m: float -- robot length in meters (default 0.73)
    cart_length_m: float -- length of a cart in meters (default 0.81)
    distance_to_cart_m: float -- used for calculating the predocking distance in front of a cart,
                                 where the distance is calculated as follows
                                 (robot_length_m + cart_length_m) / 2. + distance_to_cart_m
                                 (default 1.0)

    offset_to_approach_pose_m: float -- robot-cart offset in meters (default 0.55)
    backward_vel_docking_ms: float -- maximum backward approach velocity in m/s (default 0.1)
    max_rot_vel_docking_rads: float -- maximum rotational approach velocity in rad/s (default 0.1)
    approach_x_thresh_m: float -- cart approach threshold along x in meters (default 0.05)
    approach_y_thresh_m: float -- cart approach threshold along y in meters (default 0.05)
    approach_yaw_thresh_rad: float -- cart approach threshold for the orientation
                                      in rad/s (default 0.1)


    max_coupling_attempts: int -- maximum number of cart coupling attempts before giving up (default 3)

    post_dock_forward_distance_m: float -- distance to move forward after docking in meters (default 0.2)

    @author Wouter Houtman, Santosh Thoduka, Sebastian Blumenthal
    @contact w.houtman@tue.nl, santosh.thoduka@h-brs.de, blumenthal@locomotec.com

    '''
    def __init__(self, cart_area, cart_sub_area, action_req, action_server):
        StateMachine.__init__(self, outcomes=['done', 'failed'])

        # get cart pose state params
        map_frame_name = rospy.get_param('~map_frame_name', 'map')

        # get cart setpoint state params
        robot_length_m = float(rospy.get_param('~robot_length_m', '0.73'))
        cart_length_m = float(rospy.get_param('~cart_length_m', '0.81'))
        distance_to_cart_m = float(rospy.get_param('~distance_to_cart_m', '1.0'))

        # align and approach cart state params
        offset_to_approach_pose_m = float(rospy.get_param('~offset_to_approach_pose_m', '0.55'))
        backward_vel_docking_ms = float(rospy.get_param('~backward_vel_docking_ms', '0.1'))
        max_rot_vel_docking_rads = float(rospy.get_param('~max_rot_vel_docking_rads', '0.1'))
        approach_x_thresh_m = float(rospy.get_param('~approach_x_thresh_m', '0.05'))
        approach_y_thresh_m = float(rospy.get_param('~approach_y_thresh_m', '0.05'))
        approach_yaw_thresh_rad = float(rospy.get_param('~approach_yaw_thresh_rad', '0.1'))

        # couple to cart state params
        max_coupling_attempts = int(rospy.get_param('max_coupling_attempts', '3'))

        # post dock params
        post_dock_forward_distance_m = float(rospy.get_param('~post_dock_forward_distance_m', '0.2'))

        self.userdata.cart_pose = None
        self.userdata.pre_dock_setpoint = None
        self.userdata.post_dock_setpoint = None
        self.userdata.cart_area = cart_area
        self.userdata.cart_sub_area = cart_sub_area
        self.userdata.action_req = action_req
        self.userdata.action_server = action_server
        self.userdata.area_shape = None
        self.userdata.sub_area_shape = None

        with self:
            StateMachine.add('GET_CART_POSE', GetCartPose(map_frame_name=map_frame_name),
                             transitions={'cart_found': 'GET_SETPOINT_IN_PRE_DOCK_AREA',
                                          'cart_not_found': 'LOOK_FOR_CART',
                                          'timeout': 'failed'})

            StateMachine.add('GET_SETPOINT_IN_PRE_DOCK_AREA', GetSetpointInPreDockArea(robot_length_m=robot_length_m,
                                                                                       cart_length_m=cart_length_m,
                                                                                       distance_to_cart_m=distance_to_cart_m),
                             transitions={'setpoint_found': 'GO_TO_PRE_DOCK_SETPOINT',
                                          'setpoint_unreachable': 'failed',
                                          'timeout': 'failed'})

            StateMachine.add('GO_TO_PRE_DOCK_SETPOINT', GoToPreDockSetpoint(),
                             transitions={'reached_setpoint': 'ALIGN_AND_APPROACH_CART',
                                          'setpoint_unreachable': 'failed',
                                          'timeout': 'failed'})

            StateMachine.add('ALIGN_AND_APPROACH_CART', AlignAndApproachCart(offset_to_approach_pose_m=offset_to_approach_pose_m,
                                                                             backward_vel_docking_ms=backward_vel_docking_ms,
                                                                             max_rot_vel_docking_rads=max_rot_vel_docking_rads,
                                                                             approach_x_thresh_m=approach_x_thresh_m,
                                                                             approach_y_thresh_m=approach_y_thresh_m,
                                                                             approach_yaw_thresh_rad=approach_yaw_thresh_rad),
                             transitions={'approach_succeeded': 'COUPLE_TO_CART',
                                          'cart_not_found': 'GET_CART_POSE',
                                          'cart_pose_publisher_not_available': 'failed',
                                          'timeout': 'GO_TO_PRE_DOCK_SETPOINT'})

            StateMachine.add('COUPLE_TO_CART', CoupleToCart(max_coupling_attempts=max_coupling_attempts),
                             transitions={'coupling_succeeded': 'GET_SETPOINT_IN_POST_DOCK_AREA',
                                          'coupling_failed': 'GO_TO_PRE_DOCK_SETPOINT',
                                          'cannot_switch_to_load_mode': 'failed'})

            StateMachine.add('GET_SETPOINT_IN_POST_DOCK_AREA', GetSetpointInPostDockArea(distance_to_move=post_dock_forward_distance_m),
                             transitions={'setpoint_found': 'GO_TO_POST_DOCK_SETPOINT',
                             # we consider the action a success even if it doesn't move forward post docking
                                          'setpoint_unreachable': 'done',
                                          'timeout': 'done'})

            StateMachine.add('GO_TO_POST_DOCK_SETPOINT', GoToPostDockSetpoint(),
                             transitions={'reached_setpoint': 'done',
                                          'setpoint_unreachable': 'done',
                                          'timeout': 'done'})
            #############################################################################################################
            ## Non-nominal states; i.e. states to execute when the nominal execution fails

            StateMachine.add('LOOK_FOR_CART', LookForCart(map_frame_name=map_frame_name, robot_length_m=robot_length_m),
                             transitions={'cart_found': 'GET_SETPOINT_IN_PRE_DOCK_AREA',
                                          'cart_not_found': 'failed',
                                          'timeout': 'failed'})

