# Bunker Robot Stack

This repository contains the new ROS 2 architecture for the Bunker UGV robot stack.

The goal of this repo is to reorganize the current working robot code into clear, maintainable packages. The old stack works, but most logic currently lives inside one package, bunker_core. This new repo separates the system by responsibility, similar to the race-stack architecture pattern.

## Purpose

This stack controls a Bunker UGV for field robotics tasks such as lawn coverage, cargo hauling, obstacle handling, GPS logging, and remote operator control.

The robot uses GPS, RealSense cameras, CAN bus control, object detection, path generation, and ROS 2 navigation nodes.

## Architecture Overview

The stack is organized into clear layers:

| Layer | Package | Responsibility |
| --- | --- | --- |
| Bringup | bunker_bringup | Launch files, runtime modes, robot-level configs |
| Hardware | bunker_hardware | CAN, GPS, cameras, IMU, hardware feedback |
| State Estimation | bunker_state_estimation | GPS/IMU/EKF fusion and robot state |
| Planning | bunker_planning | Perimeter recording, cargo route recording, path generation |
| Perception | bunker_perception | YOLO, depth obstacles, safety perception |
| Behavior | bunker_behavior | Robot mode/state machine |
| Navigation | bunker_navigation | Lawn and cargo waypoint following |
| Tools | bunker_tools | Discord UI, logging, webhooks, debugging |
| Messages | bunker_msgs | Custom ROS messages |

## Main Runtime Idea

The robot should run through this high-level flow:

| Step | What happens |
| --- | --- |
| 1 | Hardware nodes publish GPS, heading, camera, IMU, and CAN feedback |
| 2 | State estimation creates a clean robot state |
| 3 | Planning records or generates waypoint paths |
| 4 | Perception reports obstacles and safety state |
| 5 | Behavior decides the active robot mode |
| 6 | Navigation calculates speed and steering |
| 7 | Hardware sends the final command to the Bunker over CAN |

The low-level robot movement still ends at the CAN controller. The navigation layer publishes a movement command, and the hardware layer sends that command to the robot.

## Planned Package Layout

| Package | Contents |
| --- | --- |
| bunker_bringup | Launch files for full robot, lawn mode, cargo mode, perception, debug |
| bunker_hardware | robot_controller_node, CAN feedback, GPS, RealSense, IMU |
| bunker_state_estimation | EKF config, robot state publisher, localization helpers |
| bunker_planning | Perimeter recorder, cargo recorder, path generator |
| bunker_perception | Object detection, obstacle detection, depth processing |
| bunker_behavior | State machine for idle, record, generate, navigate, estop |
| bunker_navigation | Stanley lawn navigation, cargo path follower |
| bunker_tools | Discord bot, GPS logger, webhook, diagnostics |
| bunker_msgs | Custom message definitions |

## Operating Modes

| Mode | Purpose |
| --- | --- |
| Base System | Start hardware, GPS, cameras, CAN, and state estimation |
| Record Lawn | Record lawn perimeter and obstacle polygons |
| Generate Lawn Path | Generate border and zigzag coverage paths |
| Lawn Navigation | Follow generated lawn coverage waypoints |
| Record Cargo | Record an exact cargo route |
| Cargo Navigation | Follow a recorded cargo route |
| Perception | Test cameras, YOLO, and obstacle detection |
| Full Robot | Start the full working field stack |
| Debug Tools | Start logging, status, and operator tools |

## Migration Source

This repo is being built from the current working repository:

Kevares-Code-

The main current package being migrated is:

src/bunker_core

Legacy or alternate code may also be pulled from:

src/CODE

YOLO-related code may be pulled from:

src/yolov_ros

## Migration Plan

| Phase | Work |
| --- | --- |
| 1 | Keep the old repo working as the source of truth |
| 2 | Create the new package structure |
| 3 | Move launch files into bunker_bringup |
| 4 | Move CAN, GPS, camera, and IMU nodes into bunker_hardware |
| 5 | Move path recording and generation into bunker_planning |
| 6 | Move Stanley and cargo navigation into bunker_navigation |
| 7 | Move YOLO and obstacle code into bunker_perception |
| 8 | Create bunker_behavior state machine |
| 9 | Move Discord, logging, and webhook code into bunker_tools |
| 10 | Replace unclear standard messages with bunker_msgs |
| 11 | Clean up legacy code, hardcoded paths, and old scripts |

## Design Rules

- bunker_bringup starts systems but does not contain robot logic.
- bunker_hardware talks to physical devices but does not decide behavior.
- bunker_planning creates paths but does not drive the robot.
- bunker_perception reports obstacles but does not own robot modes.
- bunker_behavior decides what the robot is allowed to do.
- bunker_navigation follows paths and publishes movement commands.
- bunker_tools talks to humans and external services.
- bunker_msgs defines clear message contracts.

## Current Important Source Files

| Old file | Future package |
| --- | --- |
| robot_controller_node.py | bunker_hardware |
| can_feedback_node.py | bunker_hardware |
| gps_node.py | bunker_hardware or bunker_state_estimation |
| IMU_node.py | bunker_hardware |
| realsense_*_publisher_node.py | bunker_hardware |
| path_perimeter_node.py | bunker_planning |
| cargo_path_perimeter_node.py | bunker_planning |
| path_generator_node.py | bunker_planning |
| stanlynavigation_node.py | bunker_navigation |
| cargohauling_navigation_node.py | bunker_navigation |
| object_detection_node.py | bunker_perception |
| collision_avoidance_node.py | bunker_perception |
| multi_cam_collision_avoidance.py | bunker_perception |
| discord_zigzag_test_node.py | bunker_tools, then connected to bunker_behavior |
| gps_logger_node.py | bunker_tools |

## Future Robot Modes

The behavior layer should eventually own these modes:

| Mode | Meaning |
| --- | --- |
| IDLE | Robot is waiting |
| RECORD_LAWN_PERIMETER | Recording lawn boundary |
| RECORD_OBSTACLE | Recording obstacle boundary |
| GENERATE_LAWN_PATH | Creating lawn coverage path |
| READY_FOR_LAWN_NAVIGATION | Lawn path exists and robot is waiting |
| NAVIGATE_LAWN | Robot is mowing |
| RECORD_CARGO_ROUTE | Recording cargo route |
| READY_FOR_CARGO_NAVIGATION | Cargo route exists and robot is waiting |
| NAVIGATE_CARGO | Robot is following cargo path |
| OBSTACLE_CAUTION | Robot should slow down |
| ESTOP | Robot must stop immediately |
| FAULT | Robot needs operator attention |

