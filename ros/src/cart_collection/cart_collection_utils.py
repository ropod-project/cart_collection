import math
import numpy as np
import yaml

import rospy
import dynamic_reconfigure.client
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import ropod_ros_msgs.msg

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

    return setpoint

def set_dynamic_navigation_params(param_type):
    node_name = '/maneuver_navigation/TebLocalPlannerROS'
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception, e:
       rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
       return False

    params_file = rospy.get_param('~dynamic_navigation_params', 'ros/config/dynamic_navigation_params.yaml')
    params = yaml.load(open(params_file))

    try:
        config = client.update_configuration(params[param_type])
    except Exception as e:
        rospy.logerr("Failed to set dynamic reconfigure params for " + node_name)

def get_distance_to_line(p1, p2, tp):
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    numerator = np.abs((p2.y - p1.y) * tp.x - (p2.x - p1.x) * tp.y + p2.x*p1.y - p2.y*p1.x)
    denominator  = np.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)
    return numerator / denominator


def does_point_intersect_segment_on_right(point, segment_p1, segment_p2):
    # find intersection point and check if t.x is to the left

    # t.x < p1.x + ((p1.x - p1.x)*(t.y - p1.y) / (p2.y - p1.y)
    return (point.x <
            segment_p1.x +
            ((segment_p2.x - segment_p1.x) * (point.y - segment_p1.y) /
            (segment_p2.y - segment_p1.y)))

def is_point_between_points_in_y(point, segment_p1, segment_p2):
    # check if the sign of the difference between the y coordinates of the points is different
    sign1 = math.copysign(1.0, point.y - segment_p1.y)
    sign2 = math.copysign(1.0, point.y - segment_p2.y)
    if (sign1 == sign2):
        return False
    return True

def is_point_in_polygon(point, polygon):
    # Based on algorithm here:
    # http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    #
    # If a horizontal ray from the test point to the right intersects with
    # each segment of the polygon an even number of times, it is not in the polygon

    num_intersections = 0
    for idx, p in enumerate(polygon[:-1]):
        if (is_point_between_points_in_y(point, p, polygon[idx+1]) and
            does_point_intersect_segment_on_right(point, p, polygon[idx+1])):
            num_intersections += 1
    if num_intersections % 2 == 0:
        return False
    return True

def get_pose_closest_to_normal(input_pose, edge_normal):
    pose_yaw = get_yaw_from_pose(input_pose)
    quadrants = [pose_yaw, pose_yaw + np.pi/2.0, pose_yaw - np.pi/2.0, pose_yaw + np.pi]
    min_angle = 1000.0
    closest_yaw = pose_yaw
    for quad in quadrants:
        angle = np.arctan2(np.sin(quad - edge_normal), np.cos(quad - edge_normal))
        if np.abs(angle) < min_angle:
            min_angle = np.abs(angle)
            closest_yaw = quad
    closest_yaw = np.arctan2(np.sin(closest_yaw), np.cos(closest_yaw))
    output_pose = PoseStamped()
    output_pose.header = input_pose.header
    output_pose.pose.position = input_pose.pose.position
    q = quaternion_from_euler(0.0, 0.0, closest_yaw)
    output_pose.pose.orientation.x = q[0]
    output_pose.pose.orientation.y = q[1]
    output_pose.pose.orientation.z = q[2]
    output_pose.pose.orientation.w = q[3]
    return output_pose

def get_pose_perpendicular_to_edge(shape, input_pose):
    closest_edge_idx = 0
    min_dist = 1000.0;
    input_point = ropod_ros_msgs.msg.Position()
    input_point.x = input_pose.pose.position.x
    input_point.y = input_pose.pose.position.y
    for idx, v in enumerate(shape.vertices[:-1]): # last and first vertex are the same
        dist = get_distance_to_line(v, shape.vertices[idx+1], input_point)
        if dist < min_dist:
            min_dist = dist
            closest_edge_idx = idx

    # this is the edge closest to the input_pose
    v1 = shape.vertices[closest_edge_idx]
    v2 = shape.vertices[closest_edge_idx + 1]

    dx = (v2.x - v1.x)
    dy = (v2.y - v1.y)
    # normal of edge pointing inwards (assuming anticlockwise order of points)
    enx = -dy
    eny = dx

    # check if normal points inside the shape by testing
    # if we end up inside the polygon by going along the normal
    # This check is necessary because the assumption that the polygon
    # points are given in anticlockwise order is not true
    # The ordering is dependent on how the OSM map was created; hence
    # it could be either CW or CCW. If it was CW, we need to flip the normal
    edge_normal = np.arctan2(eny, enx)
    test_point = ropod_ros_msgs.msg.Position()
    test_point.x = (v1.x + v2.x) / 2.0 + 0.01*np.cos(edge_normal)
    test_point.y = (v1.y + v2.y) / 2.0 + 0.01*np.sin(edge_normal)
    if (not is_point_in_polygon(test_point, shape.vertices)):
        # the normal was pointing away from the polygon, so we need to flip it
        enx = -enx
        eny = -eny

    edge_normal = np.arctan2(eny, enx)
    # Change the direction of input_pose by increments of pi/2 until we find
    # the one closest to the normal of edge
    output_pose = get_pose_closest_to_normal(input_pose, edge_normal)

    return output_pose

def get_pose_perpendicular_to_longest_edge(shape, offset_from_edge):
    max_length = 0.0
    longest_edge_idx = 0
    for idx, v in enumerate(shape.vertices[:-1]): # last and first vertex are the same
        v2 = shape.vertices[idx+1]
        length = np.sqrt((v2.y - v.y)**2 + (v2.x - v.x)**2)
        if length > max_length:
            max_length = length
            longest_edge_idx = idx
    v1 = shape.vertices[longest_edge_idx]
    v2 = shape.vertices[longest_edge_idx + 1]

    pose = PoseStamped()
    pose.pose.position.x = (v1.x + v2.x) / 2.0
    pose.pose.position.y = (v1.y + v2.y) / 2.0
    # we get the yaw perpendicular to the edge
    # hence, x and y are swapped when calculating the arctan
    yaw = np.arctan2((v1.x - v2.x), (v1.y - v2.y))
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.x = q[1]
    pose.pose.orientation.x = q[2]
    pose.pose.orientation.x = q[3]

    output_pose = get_pose_perpendicular_to_edge(shape, pose)
    yaw = get_yaw_from_pose(output_pose)
    output_pose.pose.position.x += (offset_from_edge * np.cos(yaw))
    output_pose.pose.position.y += (offset_from_edge * np.sin(yaw))
    return output_pose
