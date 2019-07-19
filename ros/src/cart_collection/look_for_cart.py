import smach

class LookForCart(smach.State):
    def __init__(self, timeout=5.0):
        smach.State.__init__(self,
                             outcomes=['cart_found', 'cart_not_found', 'timeout'],
                             input_keys=['docking_area'],
                             output_keys=['cart_pose'])
        self.timeout = timeout

    def execute(self, userdata):
        return 'cart_not_found'
