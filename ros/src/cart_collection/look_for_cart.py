import smach
import rospy
import actionlib
import std_msgs.msg

from tf.transformations import quaternion_from_euler

from ropod_ros_msgs.msg import GetShapeAction, GetShapeGoal
from ropod_ros_msgs.msg import ObjectList, Position
from ropod_ros_msgs.srv import ToggleObjectPublisher
from maneuver_navigation.msg import Goal as ManeuverNavGoal
from maneuver_navigation.msg import Feedback as ManeuverNavFeedback

from geometry_msgs.msg import Polygon, Point32, PoseStamped, PoseWithCovarianceStamped


from cart_collection_utils import generate_points_in_polygon, filter_points_close_to_polygon, set_dynamic_navigation_params

class LookForCart(smach.State):
    '''
    Actively looks for the cart in the specified sub area by identifying cart candidates
    and moving to different viewpoints to perceive the candidates fully.
    '''
    def __init__(self, timeout=5.0,
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
        self.toggle_cart_publisher_client = rospy.ServiceProxy('toggle_cart_publisher_srv', ToggleObjectPublisher)
        self.get_objects_sub = rospy.Subscriber("ed_object_stream", ObjectList, self.entity_callback)

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

        if userdata.area_shape is None:
            rospy.logerr("Sub area shape not available")
            return 'cart_not_found'

        possible_viewpoints = self.get_viewpoints(userdata.area_shape)

        area_polygon, area_center = self.get_polygon_and_center(userdata.area_shape)
        resp = self.toggle_cart_publisher_client(publisher_type="carts", area=area_polygon, enable_publisher=True)

        if (not resp.success):
            return 'cart_not_found'

        # Get ropot pose
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time) <= self.timeout and self.robot_pose is None:
            rospy.sleep(0.1)

        if self.robot_pose is None:
            rospy.logerr("[cart_collector] Precondition for LookForCart not met: Robot pose not available. Aborting.")
            #return 'timeout'

        # reconfigure to fine grained navigation
        set_dynamic_navigation_params('omni_drive_mode')

        viewpoint_idx = 0
        cart_found = False
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time <= self.timeout):
            # check if we've seen a cart during navigation
            if self.cart_entities is not None and len(self.cart_entities.objects) > 0:
                cart_entity_pose = self.cart_entities.objects[0].pose
                cart_found = True
                self.nav_cancel_pub.publish(True)
                break

            # check if a new goal needs to be sent
            send_new_goal = False
            if self.feedback is not None:
                if self.feedback.status == ManeuverNavFeedback.SUCCESS:
                    set_dynamic_navigation_params('non_holonomic_mode')
                    send_new_goal = True
                if self.feedback.status == ManeuverNavFeedback.FAILURE_OBSTACLES:
                    set_dynamic_navigation_params('non_holonomic_mode')
                    send_new_goal = True

            # send a new goal
            if send_new_goal:
                self.nav_cancel_pub.publish(True)
                if viewpoint_idx >= len(possible_viewpoints):
                    set_dynamic_navigation_params('non_holonomic_mode')
                    return 'cart_not_found'
                viewpoint = possible_viewpoints[viewpoint_idx]
                self.send_nav_goal(viewpoint, area_center)
                viewpoint_idx += 1

        if cart_found:
            if userdata.sub_area_shape is None:
                rospy.logerr("Sub area shape not available")
                return 'cart_not_found'
            cart_pose = get_pose_perpendicular_to_edge(userdata.sub_area_shape, cart_pose)
            self.cart_pose_pub.publish(cart_pose)
            userdata.cart_pose = cart_pose
            send_feedback(userdata.action_req, userdata.action_server, Status.MOBIDIK_DETECTED)
        set_dynamic_navigation_params('non_holonomic_mode')
        return 'cart_not_found'


    def get_polygon_and_center(self, shape):
        area_center = Position()
        for v in shape.vertices[:-1]:
            area_center.x += v.x
            area_center.y += v.y
        area_center.x /= (len(shape.vertices) - 1)
        area_center.y /= (len(shape.vertices) - 1)

        area_polygon = Polygon()
        for p in shape.vertices:
            point = Point32()
            point.x = p.x
            point.y = p.y
            area_polygon.points.append(point)
        return area_polygon, area_center

    def get_viewpoints(self, shape):

        viewpoints = generate_points_in_polygon(shape.vertices)
        viewpoints = filter_points_close_to_polygon(shape.vertices,
                                                    viewpoints,
                                                    (self.robot_length_m / 2.0) + 0.1)
        return viewpoints

    def send_nav_goal(self, viewpoint, area_center):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = viewpoint.x
        goal_pose.pose.position.y = viewpoint.y
        goal_pose.header.stamp = rospy.Time.now()
        goal_pose.header.frame_id = self.map_frame_name
        yaw = np.arctan2((area_center.y - viewpoint.y), (area_center.x - viewpoint.x))
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
        self.nav_goal_pub.publish(nav_goal)


    def entity_callback(self, msg):
        self.cart_entities = msg

    def feedback_callback(self, msg):
        self.feedback = msg

    def robot_pose_callback(self, msg):
        if self.robot_pose is None:
            self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
