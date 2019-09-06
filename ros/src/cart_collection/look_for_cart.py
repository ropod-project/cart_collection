import smach
import rospy
import actionlib
import std_msgs.msg
import numpy as np

from tf.transformations import quaternion_from_euler

from ropod_ros_msgs.msg import GetObjectsAction, GetObjectsGoal
from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal
from ropod_ros_msgs.msg import ObjectList, Position
from ropod_ros_msgs.srv import ToggleObjectPublisher
from ropod_ros_msgs.msg import Status
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from geometry_msgs.msg import Polygon, Point32, PoseStamped, PoseWithCovarianceStamped


from cart_collection_utils import generate_points_in_polygon, \
                                  filter_points_close_to_polygon, \
                                  filter_points_close_to_objects, \
                                  filter_points_in_polygon, \
                                  set_dynamic_navigation_params, \
                                  get_pose_perpendicular_to_edge, \
                                  send_feedback, \
                                  get_edge_closest_to_wall

class LookForCart(smach.State):
    '''
    Actively looks for the cart in the specified sub area by selecting random viewpoints within the cart area.
    The viewpoints are selected such that they point towards the target subarea and are not too close to walls
    or other obstacles.
    A future improvement is to select viewpoints such that they are oriented towards cart candidates.
    '''
    def __init__(self, timeout=90.0,
                map_frame_name='map',
                robot_length_m=0.73):
        smach.State.__init__(self,
                             outcomes=['cart_found', 'cart_not_found', 'timeout'],
                             input_keys=['cart_area', 'cart_sub_area', 'action_req', 'action_server', 'area_shape', 'sub_area_shape'],
                             output_keys=['cart_pose', 'action_server'])
        self.timeout = rospy.Duration.from_sec(timeout)
        self.map_frame_name = map_frame_name
        self.robot_length_m = robot_length_m
        self.cart_entities = None
        self.cart_pose_pub = rospy.Publisher("cart_pose", PoseStamped, queue_size=1)
        self.toggle_cart_publisher_client = rospy.ServiceProxy('ed_toggle_object_publisher_srv', ToggleObjectPublisher)
        self.get_objects_sub = rospy.Subscriber("ed_object_stream", ObjectList, self.entity_callback)
        self.get_objects_client = actionlib.SimpleActionClient("get_objects", GetObjectsAction)

        self.feedback = None
        self.robot_pose = None
        self.nav_goal_pub = rospy.Publisher("nav_goal",
                                            ManeuverNavGoal,
                                            queue_size=1)
        self.nav_cancel_pub = rospy.Publisher("nav_cancel",
                                            std_msgs.msg.Bool,
                                            queue_size=1)
        self.nav_feedback_sub = rospy.Subscriber("nav_feedback",
                                                 ManeuverNavFeedback,
                                                 self.feedback_callback)
        self.robot_pose_sub = rospy.Subscriber("localisation_pose",
                                               PoseWithCovarianceStamped,
                                               self.robot_pose_callback)

    def execute(self, userdata):
        self.cart_entities = None

        if userdata.area_shape is None or userdata.sub_are_shape:
            rospy.logerr("Area or sub area shape not available")
            return 'cart_not_found'

        obj_action_server_available = self.get_objects_client.wait_for_server(timeout=self.timeout)
        if not obj_action_server_available:
            rospy.logwarn("[cart_collector] Cannot get entities while looking for cart")


        sub_area_polygon = self.get_polygon(userdata.sub_area_shape)
        v1, v2 = get_edge_closest_to_wall(userdata.area_shape, userdata.sub_area_shape)
        # look at viewpoint_target from all viewpoints (i.e. this is
        # used to determine the final orientation of the robot at a given viewpoint
        viewpoint_target = Position()
        viewpoint_target.x = (v1.x + v2.x) / 2.0
        viewpoint_target.y = (v1.y + v2.y) / 2.0

        resp = self.toggle_cart_publisher_client(publisher_type="carts", area=sub_area_polygon, enable_publisher=True)

        if (not resp.success):
            rospy.logerr('[cart_collector] could not toggle cart publisher')
            return 'cart_not_found'

        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for LookForCart not met: Robot pose not available. Aborting.")
            self.reset()
            return 'timeout'

        # reconfigure to fine grained navigation
        set_dynamic_navigation_params('omni_drive_mode')
        send_feedback(userdata.action_req, userdata.action_server, Status.ACTIVE_SEARCH_MODE)

        viewpoint_idx = 1
        cart_found = False
        send_new_goal = True
        start_time = rospy.Time.now()
        nav_goal_start_time = rospy.Time.now()
        nav_timeout = rospy.Duration(20.0)
        while (rospy.Time.now() - start_time <= self.timeout):
            # check if we've seen a cart during navigation
            if self.cart_entities is not None and len(self.cart_entities.objects) > 0:
                cart_entity_pose = self.cart_entities.objects[0].pose
                cart_found = True
                self.nav_cancel_pub.publish(True)
                break

            if (rospy.Time.now() - nav_goal_start_time) > nav_timeout:
                send_new_goal = True

            # send a new goal
            if send_new_goal:
                rospy.loginfo("Trying viewpoint %d" % viewpoint_idx)
                # get obstacles
                entities = self.get_entities(userdata.cart_area)
                # generate some viewpoints
                possible_viewpoints = self.get_viewpoints(userdata.area_shape, userdata.sub_area_shape, entities)
                if len(possible_viewpoints) == 0:
                    rospy.logerr('Could not generate any more viewpoints')
                    self.reset()
                    return 'cart_not_found'
                # pick the first one
                viewpoint = possible_viewpoints[0]
                rospy.loginfo("sending nav goal")
                nav_goal = self.send_nav_goal(viewpoint, viewpoint_target)
                nav_goal_start_time = rospy.Time.now()
                self.nav_goal_pub.publish(nav_goal)
                viewpoint_idx += 1

            # check if a new goal needs to be sent
            send_new_goal = False
            if self.feedback is not None:
                if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                    set_dynamic_navigation_params('non_holonomic_mode')
                    send_new_goal = True
                if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                    set_dynamic_navigation_params('non_holonomic_mode')
                    send_new_goal = True
            rospy.sleep(0.1)


        if cart_found:
            rospy.loginfo("Cart found!")
            cart_pose = get_pose_perpendicular_to_edge(userdata.sub_area_shape, cart_entity_pose)
            self.cart_pose_pub.publish(cart_pose)
            userdata.cart_pose = cart_pose
            send_feedback(userdata.action_req, userdata.action_server, Status.MOBIDIK_DETECTED)
            self.reset()
            return 'cart_found'
        self.reset()
        return 'cart_not_found'


    def reset(self):
        set_dynamic_navigation_params('non_holonomic_mode')
        self.toggle_cart_publisher_client(enable_publisher=False)

    def get_polygon(self, shape):
        '''
        Returns geometry_msgs.msg.Polygon from ropod_ros_msgs.msg.Position[]
        '''
        area_polygon = Polygon()
        for p in shape.vertices:
            point = Point32()
            point.x = p.x
            point.y = p.y
            area_polygon.points.append(point)
        return area_polygon

    def get_viewpoints(self, area_shape, sub_area_shape, objects):
        '''
        Returns list of viewpoints within area_shape excluding those which are:
        1. too close to the edges of area_shape
        2. inside sub_area_shape
        3. too close to the edges of any object
        '''

        viewpoints = generate_points_in_polygon(area_shape.vertices, 150)
        viewpoints = filter_points_close_to_polygon(area_shape.vertices,
                                                    viewpoints,
                                                    (self.robot_length_m / 2.0) + 0.25)
        viewpoints = filter_points_close_to_objects(objects,
                                                    viewpoints,
                                                    (self.robot_length_m / 2.0) + 0.25)
        viewpoints = filter_points_in_polygon(sub_area_shape.vertices,
                                              viewpoints)
        return viewpoints

    def send_nav_goal(self, viewpoint, viewpoint_target):
        '''
        Sends navigation goal to position defined by viewpoint and orientation
        defined by line from viewpoint to viewpoint_target
        '''
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = viewpoint.x
        goal_pose.pose.position.y = viewpoint.y
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.map_frame_name
        yaw = np.arctan2((viewpoint_target.y - viewpoint.y), (viewpoint_target.x - viewpoint.x))
        q = quaternion_from_euler(0.0, 0.0, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        # Send goal
        self.feedback = None
        nav_goal = ManeuverNavGoal()
        nav_goal.conf.precise_goal = False
        nav_goal.conf.use_line_planner = False
        nav_goal.conf.append_new_maneuver = False
        nav_goal.start = self.robot_pose
        nav_goal.goal = goal_pose
        return nav_goal


    def entity_callback(self, msg):
        self.cart_entities = msg

    def feedback_callback(self, msg):
        self.feedback = msg

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose

    def get_entities(self, cart_area):
        '''
        Returns list of all entities within the cart area
        '''
        goal = GetObjectsGoal()
        goal.area_id = cart_area
        goal.area_type = 'corridor'
        goal.type = '' # get all entities
        self.get_objects_client.send_goal(goal)
        result = self.get_objects_client.wait_for_result(timeout=self.timeout)
        if not result:
            rospy.logwarn("[cart_collector] Did not get any entities while looking for cart")

        res = self.get_objects_client.get_result()
        return res.objects
