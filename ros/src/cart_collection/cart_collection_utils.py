import math

import rospy
import dynamic_reconfigure.client
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped

def get_yaw_from_pose(pose):
    orientation_q = pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    return yaw

def get_setpoint_in_front_of_pose(pose, distance):
    if pose is None:
        rospy.logerr("[cart_collector] Precondition for get_setpoint_in_front_of_pose not met: Pose not found. Aborting.")
        return None

    yaw = get_yaw_from_pose(pose)

    x_cart_in_world_frame = pose.pose.position.x
    y_cart_in_world_frame = pose.pose.position.y
    x = x_cart_in_world_frame + (distance * math.cos(yaw))
    y = y_cart_in_world_frame + (distance * math.sin(yaw))

    setpoint = PoseStamped()
    setpoint.header = pose.header
    setpoint.pose.position.x = x
    setpoint.pose.position.y = y
    setpoint.pose.position.z = pose.pose.position.z
    setpoint.pose.orientation = pose.pose.orientation

    return  setpoint

def set_omni_drive_mode():
    node_name = '/maneuver_navigation/TebLocalPlannerROS'
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception, e:
       rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
       return False

    params = {"max_vel_y" : 0.5,
              "weight_kinematics_nh": 0,
              "weight_kinematics_forward_drive": 0}
    try:
        config = client.update_configuration(params)
    except Exception as e:
        rospy.logerr("Failed to set dynamic reconfigure params for " + node_name)

def reset_to_non_holonomic_mode():
    node_name = '/maneuver_navigation/TebLocalPlannerROS'
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception, e:
       rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
       return False

    params = {"max_vel_y" : 0.0,
              "weight_kinematics_nh": 1000,
              "weight_kinematics_forward_drive": 1}
    try:
        config = client.update_configuration(params)
    except Exception as e:
        rospy.logerr("Failed to set dynamic reconfigure params for " + node_name)
