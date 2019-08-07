import rospy
from smach import StateMachine

from cart_drop_off.get_pre_undock_setpoint import GetSetpointInPreUndockArea
from cart_drop_off.go_to_pre_undock_setpoint import GoToPreUndockSetpoint
from cart_drop_off.go_to_undock_setpoint import GoToUndockSetpoint
from cart_drop_off.uncouple_from_cart import UncoupleFromCart
from cart_drop_off.get_setpoint_in_post_undock_area import GetSetpointInPostUndockArea
from cart_drop_off.go_to_post_undock_setpoint import GoToPostUndockSetpoint


class CartDropOffSM(StateMachine):
    '''
    @author Wouter Houtman, Santosh Thoduka, Sebastian Blumenthal
    @contact w.houtman@tue.nl, santosh.thoduka@h-brs.de, blumenthal@locomotec.com

    '''
    def __init__(self, cart_sub_area):
        StateMachine.__init__(self, outcomes=['done', 'failed'])

        # get setpoint in preundock area state params
        map_frame_name = rospy.get_param('~map_frame_name', 'map')
        preundock_offset_m = float(rospy.get_param('~preundock_offset_m', '0.2'))

        self.userdata.cart_sub_area = cart_sub_area

        with self:
            StateMachine.add('GET_SETPOINT_IN_PRE_UNDOCK_AREA', GetSetpointInPreUndockArea(preundock_offset_m=preundock_offset_m,
                                                                                           map_frame_name=map_frame_name),
                             transitions={'setpoint_found': 'GO_TO_PRE_UNDOCK_SETPOINT',
                                          'setpoint_unreachable': 'failed',
                                          'timeout': 'failed'})

            StateMachine.add('GO_TO_PRE_UNDOCK_SETPOINT', GoToPreUndockSetpoint(),
                             transitions={'reached_setpoint': 'GO_TO_UNDOCK_SETPOINT',
                                          'setpoint_unreachable': 'failed',
                                          'timeout': 'failed'})

            StateMachine.add('GO_TO_UNDOCK_SETPOINT', GoToUndockSetpoint(),
                             transitions={'reached_setpoint': 'UNCOUPLE_FROM_CART',
                                # assume we are close enough and just drop off the cart here anyway
                                          'setpoint_unreachable': 'UNCOUPLE_FROM_CART',
                                          'timeout': 'UNCOUPLE_FROM_CART'})

            StateMachine.add('UNCOUPLE_FROM_CART', UncoupleFromCart(),
                             transitions={'uncoupling_succeeded': 'GET_SETPOINT_IN_POST_UNDOCK_AREA',
                                          'uncoupling_failed': 'failed',
                                          'cannot_switch_to_robot_mode': 'failed'})

            StateMachine.add('GET_SETPOINT_IN_POST_UNDOCK_AREA', GetSetpointInPostUndockArea(),
                             transitions={'setpoint_found': 'GO_TO_POST_UNDOCK_SETPOINT',
                             # we consider the action a success even if it doesn't move forward post docking
                                          'setpoint_unreachable': 'done',
                                          'timeout': 'done'})

            StateMachine.add('GO_TO_POST_UNDOCK_SETPOINT', GoToPostUndockSetpoint(),
                             transitions={'reached_setpoint': 'done',
                                          'setpoint_unreachable': 'done',
                                          'timeout': 'done'})
