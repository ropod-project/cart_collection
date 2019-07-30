import math
import smach
import rospy

from geometry_msgs.msg import Twist, PoseStamped
from ropod_ros_msgs.msg import ObjectList

from cart_collection.cart_collection_utils import get_setpoint_in_front_of_pose, get_yaw_from_pose

class AlignAndApproachCart(smach.State):
    def __init__(self, timeout=15.0,
                 offset_to_approach_pose_m=0.55,
                 backward_vel_docking_ms=0.1,
                 max_rot_vel_docking_rads=0.1,
                 approach_x_thresh_m=0.05,
                 approach_y_thresh_m=0.05,
                 approach_yaw_thresh_rad=0.1):
        smach.State.__init__(self, outcomes=['approach_succeeded',
                                             'cart_not_found',
                                             'timeout'])
        self.cart_pose_feedback_sub = rospy.Subscriber("cart_pose_feedback",
                                                       ObjectList,
                                                       self.cart_front_pose_callback)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.cart_approach_pose_pub = rospy.Publisher("cart_approach_pose",
                                                      PoseStamped,
                                                      queue_size=1)
        self.offset_to_front = offset_to_approach_pose_m
        self.backward_vel_docking_ms = backward_vel_docking_ms
        self.max_rot_vel_docking_rads = max_rot_vel_docking_rads
        self.approach_x_thresh_m = approach_x_thresh_m
        self.approach_y_thresh_m = approach_y_thresh_m
        self.approach_yaw_thresh_rad = approach_yaw_thresh_rad

        self.cart_front_pose = None
        self.timeout = rospy.Duration.from_sec(timeout)
        self.pose_reached =  False

    def execute(self, userdata):
        self.cart_front_pose = None
        self.pose_reached =  False

        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time <= self.timeout) and not self.pose_reached:
            if self.cart_front_pose is not None:
                cart_approach_pose = get_setpoint_in_front_of_pose(self.cart_front_pose,
                                                                   self.offset_to_front)
                self.cart_approach_pose_pub.publish(cart_approach_pose)
                (vel, self.pose_reached) = self.calculate_final_approach_velocities(cart_approach_pose)
                self.cmd_vel_pub.publish(vel)
            else:
                rospy.logwarn("[cart_collector] Precondition for AlignAndApproachCart not met: No cart_front_pose reveived so far. Retrying.")
            rospy.sleep(0.1)

        if self.pose_reached:
            return 'approach_succeeded'
        return 'timeout'

    def cart_front_pose_callback(self, msg):
        if msg.objects:
            self.cart_front_pose =  msg.objects[0].pose

        if len(msg.objects) > 1:
            rospy.logwarn("[cart_collector] Precondition for cart_front_pose_callback not met: Only one cart front beeing detected. Taking the first one.")

    def calculate_final_approach_velocities(self, pose):
        vel = Twist()
        yaw = get_yaw_from_pose(pose)

        if abs(pose.pose.position.x) < self.approach_x_thresh_m \
            and abs(pose.pose.position.y) < self.approach_y_thresh_m \
            and abs(yaw) < self.approach_yaw_thresh_rad:
            return vel, True # Reached / Done

        if abs(pose.pose.position.x) > self.approach_x_thresh_m \
            and abs(pose.pose.position.x) > abs(pose.pose.position.y):
                # magnitude of self.backward_vel_docking_ms, sign of pose.pose.position.x
                vel.linear.x = math.copysign(self.backward_vel_docking_ms, pose.pose.position.x)
                if (abs(pose.pose.position.y) > self.approach_y_thresh_m):
                    x_time = pose.pose.position.x / vel.linear.x
                    y_vel = pose.pose.position.y / x_time
                    vel.linear.y = math.copysign(y_vel, pose.pose.position.y)

        if abs(pose.pose.position.y) > self.approach_y_thresh_m \
            and abs(pose.pose.position.y) > abs(pose.pose.position.x):
                vel.linear.y = math.copysign(self.backward_vel_docking_ms, pose.pose.position.y)
                if (abs(pose.pose.position.x) > self.approach_x_thresh_m):
                    y_time = pose.pose.position.y / vel.linear.y
                    x_vel = pose.pose.position.x / y_time
                    vel.linear.x = math.copysign(x_vel, pose.pose.position.x)

        if abs(yaw) > self.approach_yaw_thresh_rad:
            # magnitude of self.max_rot_vel_docking_rads, sign of yaw
            vel.angular.z = math.copysign(self.max_rot_vel_docking_rads, yaw)

        return vel, False # Not yet done, thus False
