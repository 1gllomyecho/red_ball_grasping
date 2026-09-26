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
        self.subscription = self.create_subscription(String, 'joint_command', self.command_callback, 10)
        self.state_publisher = self.create_publisher(String, 'arm_state', 10)
        self.position_publisher = self.create_publisher(String, 'target_position', 10)

        # Random ball position
        x = random.uniform(0.4, 0.6)
        y = random.uniform(-0.15, 0.15)
        z = 0.3
        self.target_pos = [x, y, z]

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.loadURDF("plane.urdf")
        p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.3])

        self.robot_id = p.loadURDF("kuka_iiwa/kuka_with_prismatic_gripper.urdf", [0, 0, 0], useFixedBase=True)

        visual_shape = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1])
        collision_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
        p.createMultiBody(0.1, collision_shape, visual_shape, self.target_pos)

        self.waypoints = [
            np.array([self.target_pos[0], self.target_pos[1], self.target_pos[2] + 0.3]),
            np.array([self.target_pos[0], self.target_pos[1], self.target_pos[2] + 0.1]),
            np.array([self.target_pos[0], self.target_pos[1], self.target_pos[2] - 0.10]),
        ]
        self.grasping = False

        # Publish ball position so camera_publisher can draw it
        pos_msg = String()
        pos_msg.data = str(self.target_pos)
        self.position_publisher.publish(pos_msg)
        self.get_logger().info(f'Random ball position: {self.target_pos}')

    def move_to(self, joint_angles, steps=150):
        current = [p.getJointState(self.robot_id, i)[0] for i in range(7)]
        for step in range(steps + 1):
            ratio = step / steps
            for i in range(7):
                target = current[i] * (1 - ratio) + joint_angles[i] * ratio
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=target, force=200, maxVelocity=1.0)
            for _ in range(4):
                p.stepSimulation()
            time.sleep(1.0 / 120.0)

    def command_callback(self, msg):
        if msg.data == "grasp" and not self.grasping:
            self.grasping = True
            self.get_logger().info('Received grasp command, starting execution')

            # Open gripper
            p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL, targetPosition=-0.05, force=80)
            p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=0.05, force=80)
            for _ in range(50):
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

            self.get_logger().info(f"Wrist position: {p.getLinkState(self.robot_id, 6)[0]}")
            self.get_logger().info(f"Ball position: {self.target_pos}")

            orn = p.getQuaternionFromEuler([math.pi, 0, 0])

            for wp in self.waypoints:
                joint_angles = p.calculateInverseKinematics(self.robot_id, 6, wp, orn)
                self.move_to(joint_angles)

            # Close gripper
            p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL, targetPosition=0.0, force=80)
            p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=0.0, force=80)
            for _ in range(100):
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

            # Lift
            lift_pos = np.array([self.target_pos[0], self.target_pos[1], self.target_pos[2] + 0.15])
            joint_angles = p.calculateInverseKinematics(self.robot_id, 6, lift_pos, orn)
            self.move_to(joint_angles)

            # Random transport
            for attempt in range(5):
                rand_x = random.uniform(0.3, 0.7)
                rand_y = random.uniform(-0.3, 0.3)
                rand_z = random.uniform(0.3, 0.6)
                random_pos = np.array([rand_x, rand_y, rand_z])
                joint_angles = p.calculateInverseKinematics(self.robot_id, 6, random_pos, orn)
                if joint_angles is not None:
                    self.get_logger().info(f'Random target: {random_pos}')
                    self.move_to(joint_angles)
                    break

            # Open gripper to release
            p.setJointMotorControl2(self.robot_id, 8, p.POSITION_CONTROL, targetPosition=-0.05, force=80)
            p.setJointMotorControl2(self.robot_id, 9, p.POSITION_CONTROL, targetPosition=0.05, force=80)
            for _ in range(50):
                p.stepSimulation()
                time.sleep(1.0 / 240.0)

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
