#!/bin/bash
rostopic pub -1 /collect_cart/goal ropod_ros_msgs/DockActionGoal "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  action:
    action_id: ''
    type: '$1'
    areas:
    - id: '$2'
      name: ''
      type: ''
      floor_number: 0
      sub_areas:
      - name: ''
        id: '$3'
        floor_number: 0
        type: ''
        capacity: 0
        waypoint_pose:
          position: {x: 0.0, y: 0.0, z: 0.0}
          orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
        geometry:
          vertices:
          - {x: 0.0, y: 0.0}
        right:
          point1: {x: 0.0, y: 0.0}
          point2: {x: 0.0, y: 0.0}
        left:
          point1: {x: 0.0, y: 0.0}
          point2: {x: 0.0, y: 0.0}
        turn_point: {x: 0.0, y: 0.0}
    sub_areas:
    - name: ''
      id: ''
      floor_number: 0
      type: ''
      capacity: 0
      waypoint_pose:
        position: {x: 0.0, y: 0.0, z: 0.0}
        orientation: {x: 0.0, y: 0.0, z: 0.0, w: 0.0}
      geometry:
        vertices:
        - {x: 0.0, y: 0.0}
      right:
        point1: {x: 0.0, y: 0.0}
        point2: {x: 0.0, y: 0.0}
      left:
        point1: {x: 0.0, y: 0.0}
        point2: {x: 0.0, y: 0.0}
      turn_point: {x: 0.0, y: 0.0}
    elevator: {elevator_id: 0, door_id: 0}
    start_floor: 0
    goal_floor: 0
    execution_status: ''
    estimated_duration: 0.0"
