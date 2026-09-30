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

        # Front wall (y+)
        p.createMultiBody(
            0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0], box_pos[1] + box_width/2, box_pos[2]]
        )
        # Back wall (y-)
        p.createMultiBody(
            0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0], box_pos[1] - box_width/2, box_pos[2]]
        )
        # Left wall (x+)
        p.createMultiBody(
            0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0] + box_length/2, box_pos[1], box_pos[2]]
        )
        # Right wall (x-)
        p.createMultiBody(
            0,
            p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]),
            p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]),
            basePosition=[box_pos[0] - box_length/2, box_pos[1], box_pos[2]]
        )

        self.box_pos = box_pos
        self.box_length = box_length
        self.box_width = box_width

        # ===== Random ball positions (avoid box, avoid overlap) =====
        self.ball_positions = []
        while len(self.ball_positions) < 3:
            x = random.uniform(0.45, 0.65)
            y = random.uniform(-0.2, 0.3)
            pos = [x, y, 0.3]

            too_close = False
            for existing in self.ball_positions:
                dist = np.linalg.norm(np.array(pos[:2]) - np.array(existing[:2]))
                if dist < 0.1:
                    too_close = True
                    break

            in_box = (self.box_pos[0] - self.box_length/2 <= x <= self.box_pos[0] + self.box_length/2) and \
                     (self.box_pos[1] - self.box_width/2 <= y <= self.box_pos[1] + self.box_width/2)

            if not too_close and not in_box:
                self.ball_positions.append(pos)

        self.target_pos = self.ball_positions[0]

        # ===== Create balls =====
        self.ball_ids = []
        for pos in self.ball_positions:
            visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1])
            collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
            ball_id = p.createMultiBody(0.1, collision_shape, visual_shape, pos)
            p.changeDynamics(ball_id, -1, linearDamping=0.9, angularDamping=0.9)
            self.ball_ids.append(ball_id)

        for i, ball_id in enumerate(self.ball_ids):
            pos, _ = p.getBasePositionAndOrientation(ball_id)
            self.get_logger().info(f'Ball {i} position={pos}')

        self.grasping = False
        self.success_count = 0
        self.fail_count = 0

        pos_msg = String()
        pos_msg.data = str(self.target_pos)
        self.position_publisher.publish(pos_msg)
        self.get_logger().info(f'First ball position: {self.target_pos}')

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
        return [
            np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.3]),
            np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.15]),
            np.array([ball_pos[0], ball_pos[1], ball_pos[2] - 0.08]),
            np.array([ball_pos[0], ball_pos[1], ball_pos[2] - 0.10]),
        ]

    def command_callback(self, msg):
        if msg.data == "grasp" and not self.grasping:
            self.grasping = True
            self.get_logger().info('Received grasp command, starting execution')

            orn = p.getQuaternionFromEuler([math.pi, 0, 0])

            for ball_index, ball_pos in enumerate(self.ball_positions):
                self.get_logger().info(f'=== Grasping ball {ball_index + 1} at {ball_pos} ===')
                self.target_pos = ball_pos

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
                    joint_angles = p.calculateInverseKinematics(self.robot_id, 6, wp, orn, restPoses=current)
                    self.move_to(joint_angles, gripper_target=-0.05)

                # Close gripper
                for _ in range(100):
                    p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                            targetPosition=0.0, force=500)
                    p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                            targetPosition=0.0, force=500)
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)

                # Check distance from finger joints to ball
                left_pos = p.getLinkState(self.robot_id, 8)[0]
                right_pos = p.getLinkState(self.robot_id, 9)[0]
                ball_pos_now, _ = p.getBasePositionAndOrientation(self.ball_ids[ball_index])

                d_left = np.linalg.norm(np.array(left_pos) - np.array(ball_pos_now))
                d_right = np.linalg.norm(np.array(right_pos) - np.array(ball_pos_now))
                distance = (d_left + d_right) / 2

                self.get_logger().info(f'Ball {ball_index + 1}: distance to gripper = {distance:.3f}')

                if distance < 0.08:
                    self.success_count += 1
                    self.get_logger().info(f'Ball {ball_index + 1} grasped successfully')
                else:
                    self.fail_count += 1
                    self.get_logger().info(f'Ball {ball_index + 1} grasp failed')

                # Lift
                lift_pos = np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.15])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(self.robot_id, 6, lift_pos, orn, restPoses=current)
                self.move_to(joint_angles, gripper_target=0.0)

                # Move to box (above box center, then lower)
                box_above = np.array([self.box_pos[0], self.box_pos[1], self.box_pos[2] + 0.25])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(self.robot_id, 6, box_above, orn, restPoses=current)
                self.move_to(joint_angles, gripper_target=0.0)

                box_drop = np.array([self.box_pos[0], self.box_pos[1], self.box_pos[2] + 0.15])
                current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
                joint_angles = p.calculateInverseKinematics(self.robot_id, 6, box_drop, orn, restPoses=current)
                self.move_to(joint_angles, gripper_target=0.0)

                # Open gripper to release
                for _ in range(50):
                    p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL,
                                            targetPosition=-0.05, force=500)
                    p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL,
                                            targetPosition=0.05, force=500)
                    p.stepSimulation()
                    time.sleep(1.0 / 240.0)
                
            # Return to home position
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
