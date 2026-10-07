# Red Ball Grasping Demo: KUKA + ROS2 + PyBullet Grasping Demo

A ROS2-based robotic grasping system that uses computer vision to detect a
target, computes inverse kinematics, and controls a KUKA iiwa arm with a
prismatic gripper in PyBullet simulation.

## Architecture

The system consists of three ROS2 nodes:

- `robot_driver`: places 3 strawberries in PyBullet, sends the stem position to
  `target_position`, receives pixel coordinates from `joint_command`, converts
  pixels back to world coordinates, and executes pick-and-place.
- `camera_publisher`: subscribes to `target_position`, projects the stem world
  position to pixel coordinates, draws a green stem on a synthetic image, and
  publishes to `camera_image`.
- `vision_to_arm`: subscribes to `camera_image`, detects the green stem, and
  publishes pixel coordinates to `joint_command`.

Data flow:

    robot_driver  --target_position-->  camera_publisher
    camera_publisher  --camera_image-->  vision_to_arm
    vision_to_arm  --joint_command-->  robot_driver

Key point: `robot_driver` does **not** use its own known ball positions for
grasping. Grasping coordinates come from pixel coordinates sent by
`vision_to_arm`, converted back to world coordinates via inverse projection.

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

1. `robot_driver` randomly places 3 strawberry models within a reachable
   workspace, avoiding the box area. Each strawberry consists of a red fruit
   and a green stem.
2. `robot_driver` publishes the stem world position to `target_position`.
3. `camera_publisher` projects the stem world position to pixel coordinates
   using a standard pinhole camera model, and draws a green stem on a synthetic
   black image.
4. `vision_to_arm` detects the green stem, computes its centroid pixel, and
   publishes `"u,v"` to `joint_command`.
5. `robot_driver` receives the pixel, converts it back to world coordinates
   using inverse projection with a fixed depth, then executes pick-and-place:
   open gripper, approach, descend, close gripper, lift, carry above box,
   release.
6. After each ball, `robot_driver` publishes the next stem position to
   `target_position` and the loop repeats.

## Camera Model

The synthetic camera uses a standard pinhole model:

    u = fx * X / Z + cx
    v = fy * Y / Z + cy

With:

- fx = fy = 50.0
- cx = 160.0, cy = 120.0 (320x240 image center)
- Fixed depth Z = 0.35 (stem height)

Inverse projection:

    X = (u - cx) * Z / fx
    Y = (v - cy) * Z / fy

## Known Limitations

- **Synthetic vision input**: PyBullet's `getCameraImage` does not work under
  the tested virtualized environment (VMware + NVIDIA driver combination), so
  the visual input is simulated with OpenCV. On a native Linux machine with a
  physical GPU, this can be replaced by real camera input with minimal changes.
- **Fixed depth**: The inverse projection assumes all stems are at Z = 0.35.
  If the actual Z differs, X and Y will have errors.
- **No camera extrinsics**: The camera is assumed to be at the robot base
  origin, with no rotation or translation. Real cameras require calibration.
- **Inverse kinematics**: PyBullet's `calculateInverseKinematics` may return
  unnatural arm poses for some targets. Joint limit checks are added to catch
  out-of-range solutions.
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

### 2026-10.05 - 10.07
- **Fake-image driven grasping**: `vision_to_arm` now sends pixel coordinates
  instead of a `"grasp"` signal. `robot_driver` converts pixels back to world
  coordinates via inverse projection.
- `camera_publisher` now draws the green stem (not the red fruit) using a
  standard pinhole camera model, replacing the hand-coded linear mapping.
- `vision_to_arm` now detects green (stem) instead of red (fruit), and sends
  `"u,v"` pixel coordinates.
- Added `pixel_to_world`, `check_joint_limits`, `move_to_cartesian`, and
  `send_stem_to_camera` helper functions.
- Added joint limit checking after IK to catch out-of-range solutions.
- Added `last_pixel` deduplication in `robot_driver` to ignore stale pixels.
- Lowered `extra_lift` from +0.4 to +0.25 to avoid joint 5 limit violation.
- Lowered the last grasping waypoint from +0.18 to +0.05 so the gripper
  actually descends to stem height.

## Test Results

Tested on 2026-10-07: 3 consecutive runs, all 3 strawberries grasped and placed
in the box in each run. Success rate: 100% (3/3 runs).

## License

MIT
