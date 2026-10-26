#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""机械臂抓取草莓并投放。

抓取流程（本轮修改的部分）：
    抓取成功 -> 竖直抬升 -> 保持高度水平直线平移到箱子正上方
             -> 竖直下降 -> 张开夹爪投放 -> 竖直退回

依赖：rclpy、pybullet、numpy
"""

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
    # ---- 机器人结构常量（避免满篇魔法数字）----
    ARM_JOINTS = tuple(range(7))      # 0~6 臂关节
    EE_LINK = 6                       # 末端法兰
    GRIPPER_LINK_L = 8                # 左指
    GRIPPER_LINK_R = 9                # 右指

    # ---- 运动参数 ----
    LIFT_HEIGHT = 0.20                # 抓取后先竖直抬升的高度 (m)
    DROP_CLEARANCE = 0.18             # 投放点相对箱口的高度 (m)：留够指爪长度，在空中松手让果实落进去
    RETREAT_HEIGHT = 0.15             # 投放后竖直退回高度 (m)
    LINEAR_STEP = 0.004               # 直线插值步长 (m)
    POS_TOLERANCE = 0.05              # 直线运动终点允许误差 (m)

    def __init__(self):
        super().__init__('robot_driver')
        self.home_joint_angles = [0, 0, 0, 0, 0, 0, 0]
        self.subscription = self.create_subscription(String, 'joint_command', self.command_callback, 10)
        self.state_publisher = self.create_publisher(String, 'arm_state', 10)
        self.position_publisher = self.create_publisher(String, 'target_position', 10)

        self.fx = 50.0
        self.fy = 50.0
        self.cx = 160.0
        self.cy = 120.0
        self.fixed_z = 0.35

        p.connect(p.GUI)
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(1.0 / 240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        plane_id = p.loadURDF("plane.urdf")
        p.changeDynamics(plane_id, -1, lateralFriction=1.0)
        p.resetDebugVisualizerCamera(1.5, 45, -30, [0.5, 0, 0.3])
        self.robot_id = p.loadURDF("kuka_iiwa/kuka_with_prismatic_gripper.urdf", [0, 0, 0], useFixedBase=True)

        # 关节限位只读一次并缓存：move_linear 每几毫米就要查一次，不能每次调 getJointInfo
        self.joint_limits = []
        for i in self.ARM_JOINTS:
            info = p.getJointInfo(self.robot_id, i)
            lower, upper = info[8], info[9]
            if lower > upper:          # 连续关节在 URDF 里写作 (0, -1)
                lower, upper = -math.inf, math.inf
            self.joint_limits.append((lower, upper))

        # ===== Box =====
        box_pos = [0.3, 0.5, 0.045]
        box_length = 0.1
        box_width = 0.1
        box_height = 0.15
        wall_thickness = 0.005

        p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]), p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]), basePosition=[box_pos[0], box_pos[1] + box_width/2, box_pos[2]])
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2]), p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, wall_thickness/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]), basePosition=[box_pos[0], box_pos[1] - box_width/2, box_pos[2]])
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]), p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]), basePosition=[box_pos[0] + box_length/2, box_pos[1], box_pos[2]])
        p.createMultiBody(0, p.createCollisionShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2]), p.createVisualShape(p.GEOM_BOX, halfExtents=[wall_thickness/2, box_width/2, box_height/2], rgbaColor=[0, 0.5, 1, 1]), basePosition=[box_pos[0] - box_length/2, box_pos[1], box_pos[2]])

        self.box_pos = box_pos
        self.box_length = box_length
        self.box_width = box_width
        self.box_height = box_height

        # ===== 随机草莓位置 =====
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
        self.ball_ids = []
        self.stem_ids = []
        self.fix_constraints = []

        for pos in self.ball_positions:
            fruit_visual = p.createVisualShape(p.GEOM_SPHERE, radius=0.03, rgbaColor=[1, 0, 0, 1])
            fruit_collision = p.createCollisionShape(p.GEOM_SPHERE, radius=0.03)
            fruit_id = p.createMultiBody(0.1, fruit_collision, fruit_visual, basePosition=pos)
            p.changeDynamics(fruit_id, -1, linearDamping=0.9, angularDamping=0.9)

            stem_visual = p.createVisualShape(p.GEOM_CYLINDER, radius=0.005, length=0.04, rgbaColor=[0, 1, 0, 1])
            stem_collision = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.005, height=0.04)
            stem_pos = [pos[0], pos[1], pos[2] + 0.05]
            stem_id = p.createMultiBody(0.01, stem_collision, stem_visual, basePosition=stem_pos)
            p.changeDynamics(stem_id, -1, linearDamping=0.9, angularDamping=0.9)

            p.createConstraint(fruit_id, -1, stem_id, -1, p.JOINT_FIXED, [0,0,0], [0,0,0.05], [0,0,0])
            p.setCollisionFilterPair(fruit_id, stem_id, -1, -1, 0)
            fix = p.createConstraint(fruit_id, -1, -1, -1, p.JOINT_FIXED, [0,0,0], [0,0,0], pos)
            self.fix_constraints.append(fix)
            self.ball_ids.append(fruit_id)
            self.stem_ids.append(stem_id)

        self.grasping = False
        self.success_count = 0
        self.fail_count = 0
        self.current_ball_index = 0
        self.last_pixel = None
        self.waiting_for_new_pixel = False

        self.send_stem_to_camera(self.current_ball_index)

        # 相机可能在订阅建好之前收不到第一条 target_position（DDS 发现要几百毫秒）。
        # 发丢了，相机就会画出一个假像素，第一颗球必然空抓。
        self.pixel_seen = False
        self.stem_watchdog = self.create_timer(0.5, self._resend_stem_if_no_pixel)

    # ------------------------------------------------------------------
    # 基础工具
    # ------------------------------------------------------------------
    def send_stem_to_camera(self, ball_index):
        ball_pos = self.ball_positions[ball_index]
        stem_world = [ball_pos[0], ball_pos[1], ball_pos[2] + 0.05]
        msg = String()
        msg.data = str(stem_world)
        self.position_publisher.publish(msg)
        self.get_logger().info(f'Sent stem to camera for ball {ball_index}: {stem_world}')

    def _resend_stem_if_no_pixel(self):
        if self.grasping or self.pixel_seen:
            return
        if self.current_ball_index >= len(self.ball_positions):
            return
        self.send_stem_to_camera(self.current_ball_index)

    def pixel_to_world(self, u, v):
        Z = self.fixed_z
        X = (u - self.cx) * Z / self.fx
        Y = (v - self.cy) * Z / self.fy
        return np.array([X, Y, Z])

    def current_arm_joints(self):
        return [p.getJointState(self.robot_id, i)[0] for i in self.ARM_JOINTS]

    def ee_position(self):
        return np.array(p.getLinkState(self.robot_id, self.EE_LINK)[0], dtype=float)

    def check_joint_limits(self, joint_angles):
        for i, (lower, upper) in enumerate(self.joint_limits):
            if joint_angles[i] < lower - 0.01 or joint_angles[i] > upper + 0.01:
                return False
        return True

    def command_gripper(self, target):
        """target: 0.0 闭合, -0.05 张开（与原脚本的符号约定一致）"""
        p.setJointMotorControl2(self.robot_id, self.GRIPPER_LINK_L, p.POSITION_CONTROL,
                                targetPosition=target, force=500)
        p.setJointMotorControl2(self.robot_id, self.GRIPPER_LINK_R, p.POSITION_CONTROL,
                                targetPosition=-target, force=500)

    def hold_gripper(self, target, steps=50):
        for _ in range(steps):
            self.command_gripper(target)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

    # ------------------------------------------------------------------
    # 运动
    # ------------------------------------------------------------------
    def move_to(self, joint_angles, steps=150, gripper_target=0.0):
        """关节空间插值移动（点到点，轨迹是弧线，不保证末端走直线）。"""
        current = self.current_arm_joints()
        for step in range(steps + 1):
            ratio = step / steps
            for i in self.ARM_JOINTS:
                target = current[i] * (1 - ratio) + joint_angles[i] * ratio
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=target, force=200, maxVelocity=1.0)
            self.command_gripper(gripper_target)
            for _ in range(4):
                p.stepSimulation()
            time.sleep(1.0 / 120.0)

    def move_to_cartesian(self, target_pos, orn, gripper_target=0.0):
        """求解一次 IK，然后关节空间走到该解（适合远距离快速就位）。"""
        current = self.current_arm_joints()
        joint_angles = p.calculateInverseKinematics(
            self.robot_id, self.EE_LINK, np.asarray(target_pos, dtype=float).tolist(), list(orn),
            restPoses=current, maxNumIterations=200, residualThreshold=1e-5)
        joint_angles = [float(v) for v in list(joint_angles)[:7]]
        if not self.check_joint_limits(joint_angles):
            self.get_logger().warn(f'IK 解超出关节限位，放弃目标点 {np.round(target_pos, 3)}')
            return False
        self.move_to(joint_angles, gripper_target=gripper_target)
        return True

    def move_linear(self, start_pos, end_pos, orn, gripper_target=0.0,
                    max_step=None, substeps=6):
        """末端沿直线运动：每 max_step 采一个路点，逐点 IK + 位置伺服。

        与 move_to_cartesian 的区别是这里每一小步都重解 IK，
        所以末端轨迹是直线而不是弧线，且全程保持姿态 orn。
        返回 True 表示走完且终点误差在 POS_TOLERANCE 内。
        """
        start = np.asarray(start_pos, dtype=float).reshape(3)
        end = np.asarray(end_pos, dtype=float).reshape(3)
        step_len = max_step if max_step else self.LINEAR_STEP
        distance = float(np.linalg.norm(end - start))
        n_steps = max(1, int(math.ceil(distance / step_len)))

        target = end
        for k in range(1, n_steps + 1):
            target = start + (end - start) * (k / n_steps)
            joint_angles = p.calculateInverseKinematics(
                self.robot_id, self.EE_LINK, target.tolist(), list(orn),
                restPoses=self.current_arm_joints(),
                maxNumIterations=200, residualThreshold=1e-6)
            joint_angles = [float(v) for v in list(joint_angles)[:7]]
            if not self.check_joint_limits(joint_angles):
                self.get_logger().warn(
                    f'直线运动在第 {k}/{n_steps} 步超出关节限位，中止该段')
                return False
            for i, q in enumerate(joint_angles):
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=q, force=200, maxVelocity=1.0)
            self.command_gripper(gripper_target)
            for _ in range(substeps):
                p.stepSimulation()
            time.sleep(substeps / 240.0)

        # 末端再稳一下，然后检查是否真的到位（超工作空间时 IK 会给出“够不着”的解）
        for _ in range(24):
            for i, q in enumerate(joint_angles):
                p.setJointMotorControl2(self.robot_id, i, p.POSITION_CONTROL,
                                        targetPosition=q, force=200, maxVelocity=1.0)
            self.command_gripper(gripper_target)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

        error = float(np.linalg.norm(self.ee_position() - end))
        if error > self.POS_TOLERANCE:
            self.get_logger().warn(f'直线运动终点偏差 {error * 100:.1f} cm，目标可能超出工作空间')
            return False
        return True

    # ------------------------------------------------------------------
    # 抓取后的放置动作
    # ------------------------------------------------------------------
    def _ik_hits(self, x, y, z, orn, rest):
        """纯运动学试算：末端能否以 orn 姿态精确到达 (x, y, z)。"""
        q = p.calculateInverseKinematics(
            self.robot_id, self.EE_LINK, [float(x), float(y), float(z)], list(orn),
            restPoses=rest, maxNumIterations=300, residualThreshold=1e-6)
        q = [float(v) for v in list(q)[:7]]
        if not self.check_joint_limits(q):
            return False
        for i, v in enumerate(q):
            p.resetJointState(self.robot_id, i, v)
        # IK 目标与 link6 坐标之间约有 2cm 的系统偏差，容差放宽到这里
        return np.linalg.norm(self.ee_position() - [x, y, z]) < 0.03

    def reachable_ceiling(self, x, y, orn, z_from=0.85, z_min=0.20, step=0.01):
        """末端在 (x, y) 上方、不高于 z_from 的最高可到高度。

        只做 IK + 正解，不推进物理步；试探前后恢复关节角，
        所以可以在“夹着果实”的状态下安全调用。
        """
        saved = self.current_arm_joints()
        try:
            z = z_from
            while z > z_min:
                if self._ik_hits(x, y, z, orn, saved):
                    return float(z)
                z -= step
            return None
        finally:
            for i, v in enumerate(saved):
                p.resetJointState(self.robot_id, i, v)

    def transit_height(self, start, orn):
        """抬升/平移共用的高度：想抬 LIFT_HEIGHT，但受工作空间限制就压低。

        直线路径上的每一点都要检查一次，只看两端会漏掉中段够不着的情况。
        """
        desired = min(start[2] + self.LIFT_HEIGHT, 0.85)
        ceilings = []
        for t in (0.0, 0.25, 0.5, 0.75, 1.0):
            x = start[0] + (self.box_pos[0] - start[0]) * t
            y = start[1] + (self.box_pos[1] - start[1]) * t
            ceiling = self.reachable_ceiling(x, y, orn, z_from=desired)
            if ceiling is not None:
                ceilings.append(ceiling)

        z = (min(ceilings) - 0.02) if ceilings else start[2]
        if z < start[2] + 0.02:
            self.get_logger().warn(
                f'工作空间受限（路径上最高约 {z:.2f} m），平移到箱子上方时基本贴着当前高度走')
            z = max(z, start[2])
        return z

    def place_into_box(self, orn):
        """抓取成功后的动作序列：抬升 -> 水平平移 -> 下降 -> 投放 -> 退回。"""
        start = self.ee_position()
        drop_pos = np.array([
            self.box_pos[0],
            self.box_pos[1],
            self.box_pos[2] + self.box_height / 2 + self.DROP_CLEARANCE,
        ])
        box_xy = np.array([self.box_pos[0], self.box_pos[1]])

        transit_z = self.transit_height(start, orn)

        # 1) 先竖直抬起，抬到高于其他果实的高度，避免水平移动时扫到它们
        lift_pos = np.array([start[0], start[1], transit_z])
        if not self.move_linear(start, lift_pos, orn, gripper_target=0.0):
            self.get_logger().warn('竖直抬升未完全到位，继续尝试平移')

        # 2) 核心：高度不变，只改 x/y，沿直线平移到箱子正上方。
        #    工作空间边缘处跟踪误差会变大，失败就降 5cm 重试。
        translated = False
        for z in (transit_z, transit_z - 0.05, transit_z - 0.10):
            if z < drop_pos[2] + 0.02 and z < transit_z - 1e-6:
                break
            cur = self.ee_position()
            if abs(cur[2] - z) > 0.005:
                self.move_linear(cur, np.array([cur[0], cur[1], z]), orn, gripper_target=0.0)
            cur = self.ee_position()
            if self.move_linear(cur, np.array([box_xy[0], box_xy[1], z]), orn, gripper_target=0.0):
                translated = True
                break
            self.get_logger().warn(f'高度 {z:.2f} m 处平移未到位，降低高度重试')

        if not translated:
            self.get_logger().warn('直线平移始终未到位，改走关节空间兜底')
            self.move_to_cartesian(np.array([box_xy[0], box_xy[1], self.ee_position()[2]]),
                                   orn, gripper_target=0.0)

        # 3) 竖直下降到箱口上方
        self.move_linear(self.ee_position(), drop_pos, orn, gripper_target=0.0)

        # 4) 张开夹爪，让果实落进箱子
        self.hold_gripper(-0.05, steps=120)
        time.sleep(0.3)
        for _ in range(24):
            p.stepSimulation()
            time.sleep(1.0 / 240.0)

        # 5) 竖直退回，避免张开的夹爪刮到箱壁
        retreat = self.ee_position() + np.array([0.0, 0.0, self.RETREAT_HEIGHT])
        self.move_linear(self.ee_position(), retreat, orn, gripper_target=-0.05)

    def get_waypoints(self, ball_pos):
        stem_pos = [ball_pos[0], ball_pos[1], ball_pos[2] + 0.05]
        return [
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.3]),
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.15]),
            np.array([stem_pos[0], stem_pos[1], stem_pos[2] + 0.14]),
        ]

    # ------------------------------------------------------------------
    # 指令回调
    # ------------------------------------------------------------------
    def command_callback(self, msg):
        if self.grasping:
            return
        try:
            u_str, v_str = msg.data.split(',')
            u = float(u_str)
            v = float(v_str)
        except Exception:
            return

        if self.waiting_for_new_pixel:
            self.waiting_for_new_pixel = False
            self.get_logger().info(f'Ignoring stale pixel ({u:.2f}, {v:.2f})')
            return

        if self.last_pixel is not None and abs(u - self.last_pixel[0]) < 1.0 and abs(v - self.last_pixel[1]) < 1.0:
            return
        self.last_pixel = (u, v)

        # 三颗球都处理完了，但相机每 3 秒还在发像素。没有这一句，第 4 次回调
        # 会让 ball_index 越界，在 removeConstraint 处 IndexError 崩掉整个节点。
        if self.current_ball_index >= len(self.ball_positions):
            self.grasping = False
            return

        self.pixel_seen = True
        self.grasping = True
        ball_pos = self.pixel_to_world(u, v)
        ball_index = self.current_ball_index
        orn = p.getQuaternionFromEuler([math.pi, 0, 0])

        # 预抓取点不能高于机械臂在该 XY 上实际够得到的高度，否则
        # move_to_cartesian 返回的 False 会被忽略，一路空着夹爪走过去。
        pre_z = ball_pos[2] + 0.35
        ceiling = self.reachable_ceiling(ball_pos[0], ball_pos[1], orn, z_from=pre_z)
        if ceiling is not None:
            pre_z = min(pre_z, ceiling - 0.02)
        pre_pos = np.array([ball_pos[0], ball_pos[1], pre_z])
        self.move_to_cartesian(pre_pos, orn, gripper_target=-0.05)

        self.hold_gripper(-0.05, steps=50)

        waypoints = self.get_waypoints(ball_pos)
        for wp in waypoints:
            self.move_to_cartesian(wp, orn, gripper_target=-0.05)

        self.hold_gripper(0.0, steps=50)

        contacts_L = p.getContactPoints(bodyA=self.robot_id, linkIndexA=self.GRIPPER_LINK_L)
        contacts_R = p.getContactPoints(bodyA=self.robot_id, linkIndexA=self.GRIPPER_LINK_R)
        total_force = (sum(c[9] for c in contacts_L) if contacts_L else 0) + (sum(c[9] for c in contacts_R) if contacts_R else 0)

        if total_force > 0.1:
            self.success_count += 1
            self.get_logger().info(f'Strawberry {ball_index + 1} success')
            p.removeConstraint(self.fix_constraints[ball_index])

            # 抓取成功后：竖直抬升 -> 水平直线平移到箱子上方 -> 下降投放
            self.place_into_box(orn)
        else:
            self.fail_count += 1
            self.get_logger().info(f'Strawberry {ball_index + 1} failed')

            self.hold_gripper(-0.05, steps=30)

            safe_retract = np.array([ball_pos[0], ball_pos[1], ball_pos[2] + 0.1])
            self.move_to_cartesian(safe_retract, orn, gripper_target=-0.05)

        self.current_ball_index += 1
        if self.current_ball_index < len(self.ball_positions):
            self.waiting_for_new_pixel = True
            self.pixel_seen = False
            self.send_stem_to_camera(self.current_ball_index)
            self.grasping = False
        else:
            self.move_to(self.home_joint_angles, gripper_target=0.0)
            self.grasping = False


def main(args=None):
    rclpy.init(args=args)
    node = RobotDriver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
