# cyberdog_tracking

## Introduction

Cyberdog_tracking is a package based on ROS2. This package mainly used to estimate the 3D position of the tracking target and provide target point for tracking_base module to plan the path and control the robot. The objects of position estimation include pedestrians and any regular and irregular targets selected by the user.

## Dependencies

In addition to the basic msg type and pkgs of ros2, the external dependencies are as follows:

- cyberdog_common
- protocol
- OpenCV4
- yaml-cpp

## Installation

You can install this package on Linux system as follows:

- create your workspace
- clone the code from git
- compile and install

```
colcon build --packages-up-to cyberdog_tracking --install-base=/opt/ros2/cyberdog/ --merge-install
```

## Usage

```
# 1. Start tracking node
ros2 launch cyberdog_tracking launch.py

# 2. Configure tracking node
ros2 service call /tracking/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 1}}"

# 3. Activate tracking node
ros2 service call /tracking/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 3}}"

# 4. Deactivate tracking node
ros2 service call /tracking/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 4}}"

# 5. Cleanup tracking node
ros2 service call /tracking/change_state lifecycle_msgs/srv/ChangeState "{transition: {id: 2}}"
```

## Node info

```
/tracking
  Subscribers:
    /camera/aligned_depth_to_extcolor/image_raw: sensor_msgs/msg/Image
    /person: protocol/msg/Person
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Publishers:
    /tracking/transition_event: lifecycle_msgs/msg/TransitionEvent
    /tracking_pose: geometry_msgs/msg/PoseStamped
    /tracking_status: protocol/msg/TrackingStatus
    /parameter_events: rcl_interfaces/msg/ParameterEvent
    /rosout: rcl_interfaces/msg/Log
  Service Servers:
    /tracking/change_state: lifecycle_msgs/srv/ChangeState
    /tracking/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /tracking/get_available_states: lifecycle_msgs/srv/GetAvailableStates
    /tracking/get_available_transitions: lifecycle_msgs/srv/GetAvailableTransitions
    /tracking/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /tracking/get_parameters: rcl_interfaces/srv/GetParameters
    /tracking/get_state: lifecycle_msgs/srv/GetState
    /tracking/get_transition_graph: lifecycle_msgs/srv/GetAvailableTransitions
    /tracking/list_parameters: rcl_interfaces/srv/ListParameters
    /tracking/set_parameters: rcl_interfaces/srv/SetParameters
    /tracking/set_parameters_atomically: rcl_interfaces/srv/SetParametersAtomically
  Service Clients:

  Action Servers:

  Action Clients:
```
