import smach

class LookForCart(smach.State):
    '''
    Actively looks for the cart in the specified sub area by identifying cart candidates
    and moving to different viewpoints to perceive the candidates fully.
    '''
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['cart_found', 'cart_not_found', 'timeout'],
                             input_keys=['cart_area'],
                             output_keys=['cart_pose'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'cart_not_found'
