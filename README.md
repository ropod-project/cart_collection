# cart_collection

## Summary

A ROS component that manages action calls for cart docking and undocking. The component subscribes to two different action types:

1. `DOCK`
2. `UNDOCK`

These actions are defined in [this PDDL domain](https://github.com/ropod-project/task-planner/blob/master/config/task_domains/agaplesion/hospital_transportation.pddl).

The actions are exposed through a single action server and are managed by a single state machine.

## Dependencies

The cart collection depends on the following other components:
* [`ropod_rod_msgs`](https://git.ropod.org/ropod/communication/ropod_ros_msgs): Contains message and action definitions used within the component
* [`maneuver_navigation`](https://github.com/ropod-project/ros-structured-nav): Handles navigation requests
* [`world_model_mediator`](https://git.ropod.org/ropod/wm/ropod_wm_mediator): Handles world model queries

## Launch file parameters

The component expects several parameters and topic remappings to be specified in the launch file, which are required by various docking and undocking states:

* `map_frame_name: str` -- name of the world frame for global localisation (default `map`)
* `get_objects: topic` -- name of an action server for querying world entities (default `/get_objects`)
* `cart_pose: topic` -- name of a topic for publishing the pose of a cart selected for docking (default `/cart_collection/selected_cart_pose`)

* `robot_length_m: float` -- robot length in meters (default `0.73`)
* `cart_length_m: float` -- length of a cart in meters (default `0.81`)
* `distance_to_cart_m: float` -- used for calculating the predocking distance in front of a cart, where the distance is calculated as `(robot_length_m + cart_length_m) / 2. + distance_to_cart_m` `(default 1.0)`
* `cart_predock_pose: topic` -- name of a topic for publishing the selected predocking pose in front of a cart (default `/cart_collection/cart_predock_pose`)

* `nav_goal: topic` -- name of a topic for publishing navigation goals (default `/route_navigation/goal`)
* `nav_feedback: topic` -- name of a topic for receiving navigation feedback (default `/route_navigation/feedback`)
* `localisation_pose: topic` -- name of a topic on which localisation pose estimates are published (default `/amcl_pose`)

* `cart_pose_feedback: topic` -- name of a topic on which cart hypotheses are published (default `/cart_plane_detector/objects`)
* `cmd_vel: topic` -- name of a topic for sending direct base velocity commands (default `/ropod/cmd_vel`)
* `cart_approach_pose: topic` -- name of a topic for advertising the approach pose of a cart (default `/cart_collection/cart_approach_pose`)
* `offset_to_approach_pose_m: float` -- robot-cart offset in meters (default `0.55`)
* `backward_vel_docking_ms: float` -- maximum backward approach velocity in m/s (default `0.1`)
* `max_rot_vel_docking_rads: float` -- maximum rotational approach velocity in rad/s (default `0.1`)
* `approach_x_thresh_m: float` -- cart approach threshold along x in meters (default `0.05`)
* `approach_y_thresh_m: float` -- cart approach threshold along y in meters (default `0.05`)
* `approach_yaw_thresh_rad: float` -- cart appproach threshold for the orientation in rad/s (default `0.1`)

* `docking_cmd: topic` -- name of a topic for sending commands to the robot's docking mechanism (default `/ropod/ropod_low_level_control/cmd_dock`)
* `docking_feedback: topic` -- name of a topic on which feedback from the robot's docking mechanism is advertised (default `/ropod/ropod_low_level_control/dockingFeedback`)
* `set_load_attached: topic` -- name of a topic for toggling whether the robot has a load or not (default `/route_navigation/set_load_attached`)
* `max_coupling_attempts: int` -- maximum number of cart coupling attempts before giving up (default `3`)
