import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import pybullet as p
import pybullet_data
import numpy as np
import time
import math
import random

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.home_joint_angles = [0, 0, 0, 0, 0, 0, 0]
        self.subscription = self.create_subscription(String, 'joint_command', self.command_callback, 10)
        self.state_publisher = self.create_publisher(String, 'arm_state', 10)
        self.position_publisher = self.create_publisher(String, 'target_position', 10)

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)

        p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.3])

        self.robot_id = p.loadURDF("kuka_iiwa/kuka_with_prismatic_gripper.urdf", [0, 0, 0], useFixedBase=True)

        # ===== Box =====
        box_pos = [0.3, 0.5, 0.045]
        box_length = 0.1
        box_width = 0.1
        box_height = 0.15
        wall_thickness = 0.005

        p.createMultiBody(0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0], box_pos[1] + box_width/2, box_pos[2]])
        p.createMultiBody(0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0], box_pos[1] - box_width/2, box_pos[2]])
        p.createMultiBody(0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0] + box_length/2, box_pos[1], box_pos[2]])
        p.createMultiBody(0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0] - box_length/2, box_pos[1], box_pos[2]])

        self.box_pos = box_pos
        self.box_length = box_length
        self.box_width = box_width

        # ===== Random strawberry positions =====
        self.ball_positions = []
        while len(self.ball_positions) < 3:
            x = random.uniform(0.4, 0.6)
            y = random.uniform(-0.3, 0.5)
            pos = [x, y, 0.3]

            too_close = False
            for existing in self.ball_positions:
                dist = np.linalg.norm(np.array(pos[:2]) - np.array(existing[:2]))
                if dist < 0.2:
                    too_close = True
                    break

            in_box = (self.box_pos[0] - self.box_length/2 <= x <= self.box_pos[0] + self.box_length/2) and \
                     (self.box_pos[1] - self.box_width/2 <= y <= self.box_pos[1] + self.box_width/2)

            if not too_close and not in_box:
                self.ball_positions.append(pos)

        self.target_pos = self.ball_positions[0]

        # ===== Create strawberries (fruit + stem), fruit fixed to world =====
        self.ball_ids = []
        self.stem_ids = []
        self.fix_constraints = []

        for pos in self.ball_positions:
            fruit_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1])
            fruit_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
            fruit_id = p.createMultiBody(0.1, fruit_collision, fruit_visual, basePosition=pos)
            p.changeDynamics(fruit_id, -1, linearDamping=0.9, angularDamping=0.9)

            # Stem radius 5mm
            stem_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.005, length=0.04, rgbaColor=[0, 1, 0, 1])
            stem_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.005, height=0.04)
            stem_pos = [pos[0], pos[1], pos[2] + 0.05]
            stem_id = p.createMultiBody(0.01, stem_collision, stem_visual, basePosition=stem_pos)
            p.changeDynamics(stem_id, -1, linearDamping=0.9, angularDamping=0.9)

            p.createConstraint(
                parentBodyUniqueId=fruit_id,
                parentLinkIndex=-1,
                childBodyUniqueId=stem_id,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0.05],
                childFramePosition=[0, 0, 0]
            )

            p.setCollisionFilterPair(fruit_id, stem_id, -1, -1, 0)

            fix = p.createConstraint(
                parentBodyUniqueId=fruit_id,
                parentLinkIndex=-1,
                childBodyUniqueId=-1,
                childLinkIndex=-1,
                jointType=p.JOINT_FIXED,
                jointAxis=[0, 0, 0],
                parentFramePosition=[0, 0, 0],
                childFramePosition=pos
            )
            self.fix_constraints.append(fix)

            self.ball_ids.append(fruit_id)
            self.stem_ids.append(stem_id)

        for i, ball_id in enumerate(self.ball_ids):
            pos, _ = p.getBasePositionAndOrientation(ball_id)
            self.get_logger().info(f'Strawberry {i} fruit position={pos}')

        self.grasping = False
        self.success_count = 0
        self.fail_count = 0

        pos_msg = String()
        pos_msg.data = str(self.target_pos)
        self.position_publisher.publish(pos_msg)
        self.get_logger().info(f'First strawberry position: {self.target_pos}')

    def move_to(self, joint_angles, steps=150, gripper_target=0.0):
        current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
        for step in range(steps + 1):
            ratio = step / steps
            for i in range(7):
                target = current[i] * (1 - ratio) + joint_angles[i] * ratio
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=target, force=200, maxVelocity=1.0)
            p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                    targetPosition=gripper_target, force=500)
            p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                    targetPosition=-gripper_target, force=500)
            for _ in range(4):
                p.stepSimulation()
            time.sleep(1.0 / 120.0)

    def get_waypoints(self, ball_pos):
        stem_pos = [ball_pos[0], ball_pos[1], ball_pos[2] + 0.05]
        return [
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.3]),
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.25]),
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.18]),
        ]

    def command_callback(self, msg):
        if msg.data == "grasp" and not self.grasping:
            self.grasping = True
            self.get_logger().info('Received grasp command, starting execution')

            orn = p.getQuaternionFromEuler([math.pi, 0, 0])

            for ball_index, ball_pos in enumerate(self.ball_positions):
                self.get_logger().info(f'=== Grasping strawberry {ball_index + 1} at {ball_pos} ===')
                self.target_pos = ball_pos

                # Pre-approach
                pre_pos = np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.35])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(
                    self.robot_id, 6, pre_pos, orn,
                    restPoses=current,
                    maxNumIterations=200,
                    residualThreshold=1e-5
                )
                self.move_to(joint_angles, gripper_target=-0.05)

                # Open gripper
                for _ in range(50):
                    p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                            targetPosition=-0.05, force=500)
                    p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                            targetPosition=0.05, force=500)
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)

                waypoints = self.get_waypoints(ball_pos)
                for wp in waypoints:
                    current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                    joint_angles = p.calculateInverseKinematics(
                        self.robot_id, 6, wp, orn,
                        restPoses=current,
                        maxNumIterations=200,
                        residualThreshold=1e-5
                    )
                    self.move_to(joint_angles, gripper_target=-0.05)

                # Close gripper
                for _ in range(100):
                    p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                            targetPosition=0.0, force=500)
                    p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                            targetPosition=0.0, force=500)
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)

                # Check distance from finger joints to STEM
                left_pos = p.getLinkState(self.robot_id, 8)[0]
                right_pos = p.getLinkState(self.robot_id, 9)[0]
                stem_pos_now, _ = p.getBasePositionAndOrientation(self.stem_ids[ball_index])

                d_left = np.linalg.norm(np.array(left_pos) - np.array(stem_pos_now))
                d_right = np.linalg.norm(np.array(right_pos) - np.array(stem_pos_now))
                distance = (d_left + d_right) / 2

                if distance < 0.1:
                    self.success_count += 1
                    self.get_logger().info(f'Strawberry {ball_index + 1} grasped successfully')
                    p.removeConstraint(self.fix_constraints[ball_index])
                else:
                    self.fail_count += 1
                    self.get_logger().info(f'Strawberry {ball_index + 1} grasp failed')

                # Lift
                # Lift
                lift_pos = np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.25])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(
                    self.robot_id, 6, lift_pos, orn,
                    restPoses=current,
                    maxNumIterations=200,
                    residualThreshold=1e-5
                )
                self.move_to(joint_angles, gripper_target=0.0)

                # Extra lift: go straight up
                extra_lift = np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.4])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(
                    self.robot_id, 6, extra_lift, orn,
                    restPoses=current,
                    maxNumIterations=200,
                    residualThreshold=1e-5
                )
                self.move_to(joint_angles, gripper_target=0.0)

                # Step 1: Move to high above box
                box_high = np.array([self.box_pos[0], self.box_pos[1], self.box_pos[2] + 0.4])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(
                    self.robot_id, 6, box_high, orn,
                    restPoses=current,
                    maxNumIterations=200,
                    residualThreshold=1e-5
                )
                self.move_to(joint_angles, gripper_target=0.0)
                # Step 2: Lower to above box opening
                box_drop = np.array([self.box_pos[0], self.box_pos[1], self.box_pos[2] + 0.325])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(
                    self.robot_id, 6, box_drop, orn,
                    restPoses=current,
                    maxNumIterations=200,
                    residualThreshold=1e-5
                )
                self.move_to(joint_angles, gripper_target=0.0)

                # Open gripper to release
                for _ in range(50):
                    p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                            targetPosition=-0.05, force=500)
                    p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                            targetPosition=0.05, force=500)
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)

            self.get_logger().info('Returning to home position')
            self.move_to(self.home_joint_angles, gripper_target=0.0)

            self.get_logger().info(f'=== Summary: {self.success_count} success, {self.fail_count} fail ===')

            state = p.getLinkState(self.robot_id, 6)
            state_msg = String()
            state_msg.data = str(list(state[0]))
            self.state_publisher.publish(state_msg)

            self.grasping = False

def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
