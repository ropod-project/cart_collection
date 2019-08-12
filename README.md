# cart_collection

## Summary

A ROS component that manages action calls for cart docking and undocking. The component subscribes to two different action types:

1. `DOCK`
2. `UNDOCK`

These actions are defined in [this PDDL domain](https://github.com/ropod-project/task-planner/blob/master/config/task_domains/agaplesion/hospital_transportation.pddl).

The actions are exposed through a single action server and are managed by two state machines.

## Dependencies

The cart collection depends on the following other components:
* [`ropod_rod_msgs`](https://git.ropod.org/ropod/communication/ropod_ros_msgs): Contains message and action definitions used within the component
* [`maneuver_navigation`](https://github.com/ropod-project/ros-structured-nav): Handles navigation requests
* [`world_model_mediator`](https://git.ropod.org/ropod/wm/ropod_wm_mediator): Handles world model queries
  * The `world_model_mediator` requests information from [`OSM`](https://git.ropod.org/ropod/wm/osm_bridge_ros_wrapper), [`ed_entity_server`](https://github.com/ropod-project/ed_entity_server) and [`cart_detection`](https://git.ropod.org/ropod/perception/cart_detection).

## Launch file parameters

The component expects several parameters and topic remappings to be specified in the launch file, which are required by various docking and undocking states:

* `dynamic_navigation_params: str` -- file containing sets of dynamic reconfigure parameters for navigation (default `ros/config/dynamic_navigation_params.yaml`)
* `map_frame_name: str` -- name of the world frame for global localisation (default `map`)
* `get_objects: topic` -- name of an action server for querying world entities (default `/get_objects`)
* `cart_pose: topic` -- name of a topic for publishing the pose of a cart selected for docking (default `/cart_collection/selected_cart_pose`)

* `robot_length_m: float` -- robot length in meters (default `0.73`)
* `cart_length_m: float` -- length of a cart in meters (default `0.81`)
* `distance_to_cart_m: float` -- used for calculating the predocking distance in front of a cart, where the distance is calculated as `(robot_length_m + cart_length_m) / 2. + distance_to_cart_m` `(default 0.4)`
* `cart_predock_pose: topic` -- name of a topic for publishing the selected predocking pose in front of a cart (default `/cart_collection/cart_predock_pose`)

* `nav_goal: topic` -- name of a topic for publishing navigation goals (default `/route_navigation/goal`)
* `nav_feedback: topic` -- name of a topic for receiving navigation feedback (default `/route_navigation/feedback`)
* `localisation_pose: topic` -- name of a topic on which localisation pose estimates are published (default `/amcl_pose`)

* `cart_pose_feedback: topic` -- name of a topic on which cart hypotheses are published (default `/cart_plane_detector/objects`)
* `toggle_cart_publisher_srv: service` -- name of a service which toggles the publishing of cart hypotheses on the `cart_pose_feedback` topic (default `/cart_plane_detector/toggle_object_publisher`)
* `cmd_vel: topic` -- name of a topic for sending direct base velocity commands (default `/ropod/cmd_vel`)
* `cart_approach_pose: topic` -- name of a topic for advertising the approach pose of a cart (default `/cart_collection/cart_approach_pose`)
* `offset_to_approach_pose_m: float` -- robot-cart offset in meters (default `0.5`)
* `backward_vel_docking_ms: float` -- maximum backward approach velocity in m/s (default `0.1`)
* `max_rot_vel_docking_rads: float` -- maximum rotational approach velocity in rad/s (default `0.1`)
* `approach_x_thresh_m: float` -- cart approach threshold along x in meters (default `0.05`)
* `approach_y_thresh_m: float` -- cart approach threshold along y in meters (default `0.05`)
* `approach_yaw_thresh_rad: float` -- cart approach threshold for the orientation in rad (default `0.1`)

* `docking_cmd: topic` -- name of a topic for sending commands to the robot's docking mechanism (default `/ropod/ropod_low_level_control/cmd_dock`)
* `docking_feedback: topic` -- name of a topic on which feedback from the robot's docking mechanism is advertised (default `/ropod/ropod_low_level_control/dockingFeedback`)
* `set_load_attached: topic` -- name of a topic for toggling whether the robot has a load or not (default `/route_navigation/set_load_attached`)
* `max_coupling_attempts: int` -- maximum number of cart coupling attempts before giving up (default `3`)

* `cart_post_dock_pose: topic` -- name of a topic for publishing the post dock pose for the robot (default `/cart_colleciton/cart_post_dock_pose`)
* `post_dock_forward_distance_m: float` -- distance in meters that the robot should move forward after docking (default `0.1`)

* `cart_pre_undock_pose: topic` -- name of a topic for publishing the pre-undock pose for the robot (default `/cart_collection/cart_pre_undock_pose`)
* `preundock_offset_m: float` -- distance from the wall (in meters) of the pre-undock pose (default `0.5`)

* `cart_undock_pose: topic` -- name of a topic for publishing the undock pose for the robot (default `/cart_collection/cart_undock_pose`)
* `undock_offset_m: float` -- distance from the wall (in meters) for the undock pose (default `0.2`)

* `cart_post_undock_pose: topic` -- name of a topic for publishing the post undock pose of the robot (default `/cart_collection/cart_post_undock_pose`)
* `post_undock_forward_distance_m` -- distance from the wall (in meters) for the post undock pose (default `0.7`)
