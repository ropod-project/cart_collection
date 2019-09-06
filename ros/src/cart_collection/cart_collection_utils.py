import math
import numpy as np
import yaml

import rospy
import dynamic_reconfigure.client
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import PoseStamped
import ropod_ros_msgs.msg


def send_feedback(action_req, action_server, feedback_status_code=0, sm_state=''):
    '''
    Publishes action feedback, setting the status code to `feedback_status_code`
    The status codes are defined in ropod_ros_msgs.msg.Status

    args:
    action_req: ropod_ros_msgs.msg.Action -- action message
    action_server: ActionServer -- action server
    feedback_status_code: uint16 ropod_ros_msgs.msg.Status.status_code -- status code for current action
    sm_state: string -- state of a state machine
    '''
    feedback = ropod_ros_msgs.msg.TaskProgressDOCK()
    feedback.action_id = action_req.action_id
    feedback.action_type = action_req.type
    feedback.status.domain = ropod_ros_msgs.msg.Status.ROBOT
    feedback.status.module_code = ropod_ros_msgs.msg.Status.MOBIDIK_COLLECTION
    feedback.status.status_code = feedback_status_code
    feedback.status.sm_state = sm_state
    feedback_msg = ropod_ros_msgs.msg.DockFeedback()
    feedback_msg.feedback = feedback
    action_server.publish_feedback(feedback_msg)


def get_yaw_from_pose(pose):
    '''
    Returns yaw (in radians) of the input pose

    args:
    pose: geometry_msgs.msg.PoseStamped -- input pose
    '''
    orientation_q = pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    _, _, yaw = euler_from_quaternion(orientation_list)
    return yaw


def get_setpoint_in_front_of_pose(pose, distance):
    '''
    Returns a pose offset by `distance` from the input pose

    args:
    pose: geometry_msgs.msg.PoseStamped -- input pose
    distance: float -- distance (in meters) to offset input pose
    '''
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
    '''
    Sets a list of dynamic reconfigure parameters defined by the string `param_type`

    args:
    param_type: str -- name of list of parameters specified in the `dynamic_navigation_params` file

    '''
    node_name = '/maneuver_navigation/TebLocalPlannerROS'
    try:
        client = dynamic_reconfigure.client.Client(node_name, timeout=1.5)
    except Exception as e:
        rospy.logerr("Service {0} does not exist".format(node_name + '/set_parameters'))
        return False

    params_file = rospy.get_param('~dynamic_navigation_params', 'ros/config/dynamic_navigation_params.yaml')
    params = yaml.load(open(params_file))

    try:
        client.update_configuration(params[param_type])
    except Exception as e:
        rospy.logerr("Failed to set dynamic reconfigure params for " + node_name)
        rospy.logerr(e.message())


def get_distance_to_line(p1, p2, tp):
    '''
    Returns the distance of point `tp` from the line defined by `p1` and `p2`

    args:
    p1: ropod_ros_msgs.msg.Position -- point 1 of the line
    p2: ropod_ros_msgs.msg.Position -- point 2 of the line
    tp: ropod_ros_msgs.msg.Position -- test point whose distance from p1-p2 is required
    '''
    # https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
    numerator = np.abs((p2.y - p1.y) * tp.x - (p2.x - p1.x) * tp.y + p2.x*p1.y - p2.y*p1.x)
    denominator = np.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)
    return numerator / denominator


def does_point_intersect_segment_on_right(point, segment_p1, segment_p2):
    '''
    Returns True if `point` intersects the line segment defined by `segment_p1` and `segment_p2` on it's right side

    args:
    point: ropod_ros_msgs.msg.Position -- test point
    segment_p1: ropod_ros_msgs.msg.Position -- point 1 of the line segment
    segment_p2: ropod_ros_msgs.msg.Position -- point 2 of the line segment
    '''
    # find intersection point and check if t.x is to the left

    # t.x < p1.x + ((p1.x - p1.x)*(t.y - p1.y) / (p2.y - p1.y)
    return (point.x <
            segment_p1.x +
            ((segment_p2.x - segment_p1.x) * (point.y - segment_p1.y) /
             (segment_p2.y - segment_p1.y)))


def is_point_between_points_in_y(point, segment_p1, segment_p2):
    '''
    Returns True if the y-coordinate of `point` is between the y-coordinates of `segment_p1` and `segment_p2`

    args:
    point: ropod_ros_msgs.msg.Position -- test point
    segment_p1: ropod_ros_msgs.msg.Position -- point 1 of the line segment
    segment_p2: ropod_ros_msgs.msg.Position -- point 2 of the line segment
    '''
    # check if the sign of the difference between the y coordinates of the points is different
    sign1 = math.copysign(1.0, point.y - segment_p1.y)
    sign2 = math.copysign(1.0, point.y - segment_p2.y)
    if (sign1 == sign2):
        return False
    return True


def is_point_in_polygon(point, polygon):
    '''
    Returns True if `point` is contained within the `polygon`

    args:
    point: ropod_ros_msgs.msg.Position -- test point
    polygon: list of ropod_ros_msgs.msg.Position -- polygon defined as a list of points; the first and last point are identical
    '''
    # Based on algorithm here:
    # http://www.eecs.umich.edu/courses/eecs380/HANDOUTS/PROJ2/InsidePoly.html
    # https://stackoverflow.com/questions/8721406/how-to-determine-if-a-point-is-inside-a-2d-convex-polygon
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

def is_pose_in_polygon(pose, polygon):
    '''
    Returns True if `pose` is contained within the `polygon`

    args:
    pose: geometry_msgs.msg.PoseStamped -- test pose
    polygon: list of ropod_ros_msgs.msg.Position -- polygon defined as a list of points; the first and last point are identical
    '''
    point = ropod_ros_msgs.msg.Position()
    point.x = pose.pose.position.x
    point.y = pose.pose.position.x
    return is_point_in_polygon(point, polygon)


def get_pose_closest_to_normal(input_pose, edge_normal):
    '''
    Returns a pose with the same position as `input_pose` but whose orientation best aligns with the `edge_normal`

    The best alignment is based on checking the angular difference between the normal and four angles
    spaced 90 degrees apart including the orientation of `input_pose`

    args:
    input_pose: geometry_msgs.msg.PoseStamped -- input pose
    edge_normal: float -- angle (in radian) of the normal of an edge / line segment
    '''
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
    '''
    Returns a pose with the same position as `input_pose` but whose orientation best aligns with the normal
    of the edge of `shape` which is closest to `input_pose`. The orientation of the output pose will face
    the inside of the polygon defined by `shape`.

    args:
    shape: ropod_ros_msgs.msg.Shape -- polygon defined by list of points
    input_pose: geometry_msgs.msg.PoseStamped -- input pose
    '''
    closest_edge_idx = 0
    min_dist = 1000.0
    input_point = ropod_ros_msgs.msg.Position()
    input_point.x = input_pose.pose.position.x
    input_point.y = input_pose.pose.position.y
    for idx, v in enumerate(shape.vertices[:-1]):  # last and first vertex are the same
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

def get_edge_closest_to_wall(area_shape, sub_area_shape):
    '''
    Returns the edge of the `sub_area_shape` (defined by two points), which is closest to
    an edge of `area_shape`. Effectively, this finds the edge of the sub area which is closest
    to a wall

    args:
    area_shape: list of ropod_ros_msgs.msg.Position -- area polygon defined by list of points
    sub_area_shape: list of ropod_ros_msgs.msg.Position -- sub area polygon defined by list of points
    '''
    # find the edge of the sub-area which is closest to one of the
    # edges of the area. We assume this to be the edge closest to a wall
    min_distance = 1000.0
    wall_edge_idx = 0
    for idx1, area_v in enumerate(area_shape.vertices[:-1]):  # last and first vertex are the same
        area_v2 = area_shape.vertices[idx1+1]
        p1 = ropod_ros_msgs.msg.Position()
        p1.x = (area_v.x + area_v2.x) / 2.0
        p1.y = (area_v.y + area_v2.y) / 2.0
        for idx2, sub_area_v in enumerate(sub_area_shape.vertices[:-1]):  # last and first vertex are the same
            sub_area_v2 = sub_area_shape.vertices[idx2+1]
            p2 = ropod_ros_msgs.msg.Position()
            p2.x = (sub_area_v.x + sub_area_v2.x) / 2.0
            p2.y = (sub_area_v.y + sub_area_v2.y) / 2.0
            dist = np.sqrt((p2.y - p1.y)**2 + (p2.x - p1.x)**2)
            if dist < min_distance:
                min_distance = dist
                wall_edge_idx = idx2
    return sub_area_shape.vertices[wall_edge_idx], sub_area_shape.vertices[wall_edge_idx + 1]


def get_pose_perpendicular_to_wall(area_shape, sub_area_shape, offset_from_edge):
    '''
    Returns a pose whose orientation best aligns with the normal
    of the edge of the sub area which is closest to an edge of the area, with
    the position offset from the edge by `offset_from_edge`.
    The orientation of the output pose will face the inside of the polygon defined by
    `sub_area_shape`.
    By finding the edge of the sub area closest to an edge of the area, we are effectively
    finding the edge closest to a wall.

    args:
    area_shape: list of ropod_ros_msgs.msg.Position -- area polygon defined by list of points
    sub_area_shape: list of ropod_ros_msgs.msg.Position -- sub area polygon defined by list of points
    offset_from_edge: float -- offset (in meters) of the pose from the selected edge of the sub area shape
    '''

    v1, v2 = get_edge_closest_to_wall(area_shape, sub_area_shape)

    pose = PoseStamped()
    pose.pose.position.x = (v1.x + v2.x) / 2.0
    pose.pose.position.y = (v1.y + v2.y) / 2.0
    # we need the yaw perpendicular to the edge
    # hence, x and y are swapped when calculating the arctan
    yaw = np.arctan2((v1.x - v2.x), (v1.y - v2.y))
    q = quaternion_from_euler(0.0, 0.0, yaw)
    pose.pose.orientation.x = q[0]
    pose.pose.orientation.x = q[1]
    pose.pose.orientation.x = q[2]
    pose.pose.orientation.x = q[3]

    output_pose = get_pose_perpendicular_to_edge(sub_area_shape, pose)
    yaw = get_yaw_from_pose(output_pose)
    output_pose.pose.position.x += (offset_from_edge * np.cos(yaw))
    output_pose.pose.position.y += (offset_from_edge * np.sin(yaw))
    return output_pose

def generate_points_in_triangle(triangle):
    '''
    Returns a list of points which are contained inside the `triangle`

    args:
    triangle: list of ropod_ros_msgs.msg.Position -- triangle defined as a list of three points
    '''

    ptriangle = []
    for p in triangle:
        ptriangle.append([p.x, p.y])
    ptriangle = np.array(ptriangle)
    origin = ptriangle[0]
    ptriangle = ptriangle - origin
    generated_points = []
    # Based on the first non-uniform example here: http://mathworld.wolfram.com/TrianglePointPicking.html
    for idx in range(60):
        a1 = np.random.random()
        a2 = np.random.random()
        p = (ptriangle[1] * a1) + (ptriangle[2] * (1.0 - a1) * a2);
        p += origin
        rosp = ropod_ros_msgs.msg.Position()
        rosp.x = p[0]
        rosp.y = p[1]
        generated_points.append(rosp)
    return generated_points



def generate_points_in_polygon(closed_polygon):
    '''
    Returns a list of points which are contained inside the `closed_polygon`

    args:
    closed_polygon: list of ropod_ros_msgs.msg.Position -- polygon defined as a list of points; the first and last point are identical
    '''

    polygon = closed_polygon[:-1]
    generated_points = []
    for idx, p in enumerate(polygon):
        triangle = [p, polygon[(idx+1) % len(polygon)], polygon[(idx+2) % len(polygon)]]
        triangle_points = generate_points_in_triangle(triangle)
        generated_points.extend(triangle_points)
    return generated_points

def filter_points_close_to_polygon(polygon, input_points, distance_threshold):
    '''
    Returns a list of points which are at least `distance_threshold` away from all edges of the `polygon`

    args:
    polygon: list of ropod_ros_msgs.msg.Position -- polygon defined as a list of points; the first and last point are identical
    input_points: list of ropod_ros_msgs.msg.Position
    distance_threshold: minimum distance (in meter) of filtered points to every edge of the polygon
    '''

    filtered_points = []
    for test_point in input_points:
        threshold_satisfied = True
        for idx, p in enumerate(polygon[:-1]):
            p2 = polygon[idx + 1]
            distance = get_distance_to_line(p, p2, test_point)
            if (distance < distance_threshold):
                threshold_satisfied = False
                break
        if threshold_satisfied:
            filtered_points.append(test_point)
    return filtered_points

def filter_points_close_to_objects(objects, input_points, distance_threshold):
    '''
    Returns a list of points which are at least `distance_threshold` away from all `objects`

    args:
    objects: list of ropod_ros_msgs.msg.Position -- polygon defined as a list of points; the first and last point are identical
    input_points: list of ropod_ros_msgs.msg.Position
    distance_threshold: minimum distance (in meter) of filtered points to every edge of the polygon
    '''

    filtered_points = []
    for test_point in input_points:
        threshold_satisfied = True
        for obj in objects:
            for idx, p in enumerate(obj.shape.polygon.points):
                p2 = obj.shape.polygon.points[(idx + 1)%len(obj.shape.polygon.points)]
                distance = get_distance_to_line(p, p2, test_point)
                if (distance < distance_threshold):
                    threshold_satisfied = False
                    break
            if not threshold_satisfied:
                break
        if threshold_satisfied:
            filtered_points.append(test_point)
    return filtered_points
