# Red Ball Grasping Demo: KUKA + ROS2 + PyBullet Grasping Demo

A ROS2-based robotic grasping system that uses computer vision to detect a
target, computes inverse kinematics, and controls a KUKA iiwa arm with a
prismatic gripper in PyBullet simulation.

## Architecture

The system consists of three ROS2 nodes:

- `camera_publisher`: generates an image containing a red ball and publishes it
  to `camera_image`.
- `vision_to_arm`: subscribes to `camera_image`, detects the red ball,
  and publishes a grasp command to `joint_command`.
- `robot_driver`: places the ball at a random position, publishes the position
  to `target_position`, receives the grasp command, and executes the full
  pick-and-place motion.

Data flow:

    robot_driver  --target_position-->  camera_publisher
    camera_publisher  --camera_image-->  vision_to_arm
    vision_to_arm  --joint_command-->  robot_driver

## Requirements

- Ubuntu 24.04 (tested on VMware)
- ROS2 Jazzy
- Python 3.12
- PyBullet
- OpenCV
- NumPy

## Build

    cd ~/ros2_ws
    colcon build --packages-select strawberry_vision
    source install/setup.bash

## Run

    ros2 launch strawberry_vision vision_launch.py

## What It Does

1. `robot_driver` randomly places 3 strawberry models within a reachable workspace, avoiding the box area. Each strawberry consists of a red fruit and a green stem.
2. The strawberry positions are published to `target_position`.
3. `camera_publisher` maps the physical position to pixel coordinates and draws the strawberries on a synthetic image.
4. `vision_to_arm` detects the red fruit and sends a grasp command.
5. `robot_driver` opens the gripper, moves through three waypoints to the stem, closes the gripper, lifts the strawberry, carries it above the box, and releases it. This repeats for all 3 strawberries, then the arm returns to home position.

## Known Limitations

- **Synthetic vision input**: PyBullet's `getCameraImage` does not work under
  the tested virtualized environment (VMware + NVIDIA driver combination), so
  the visual input is simulated with OpenCV. On a native Linux machine with a
  physical GPU, this can be replaced by real camera input with minimal changes.
- **Inverse kinematics**: PyBullet's `calculateInverseKinematics` may return
  unnatural arm poses for some targets. The current implementation relies on
  well-chosen waypoints to avoid this.
- **Gripper range**: The prismatic gripper has limited travel, so it can only
  grasp small objects.
  

## Changelog

### 2026-09-30
- Multi-ball sequential grasping (3 balls)
- Grasp result statistics (success/fail per ball)
- Box placement (drop balls into a box)
- Return to home position after all balls are done

### 2026-10-04
- Replaced red balls with strawberry models (fruit + stem)
- Grasp the stem instead of the fruit
- Fruit fixed to world (simulating stem attached to plant), released after grasp
- Extra lift before box placement
- Two-step box placement (high above box, then lower to opening)

## Test Results

Tested 3 consecutive runs, all 3 strawberries grasped and placed in the box in each run.
Success rate: 100% (3/3 runs).


## License

MIT
