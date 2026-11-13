#!/usr/bin/env python3
"""MoveIt 관절 목표 계획+실행 시험 (dsr_bringup2_moveit.launch.py 실행 중이어야 함).
사용: python3 moveit_joint_test.py 30 20 60 0 0 0   (J1~J6 목표 [deg], 생략 시 예시값)
"""
import math
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState

JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]


def main():
    deg = [float(v) for v in sys.argv[1:7]] if len(sys.argv) >= 7 else [30, 20, 60, 0, 0, 0]
    rclpy.init()
    node = Node("moveit_joint_test")
    cli = ActionClient(node, MoveGroup, "/move_action")
    if not cli.wait_for_server(timeout_sec=10):
        print("move_action 서버 없음"); return 1

    goal = MoveGroup.Goal()
    req = goal.request
    req.group_name = "manipulator"
    req.num_planning_attempts = 5
    req.allowed_planning_time = 5.0
    req.max_velocity_scaling_factor = 0.2
    req.max_acceleration_scaling_factor = 0.2
    c = Constraints()
    for name, d in zip(JOINTS, deg):
        jc = JointConstraint(joint_name=name, position=math.radians(d), tolerance_above=1e-3,
                             tolerance_below=1e-3, weight=1.0)
        c.joint_constraints.append(jc)
    req.goal_constraints.append(c)
    goal.planning_options.plan_only = False

    print(f"목표 [deg]: {deg}  (속도/가속 20%)")
    fut = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    gh = fut.result()
    if not gh.accepted:
        print("목표 거부됨"); return 1
    rfut = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rfut, timeout_sec=120)
    res = rfut.result().result
    print(f"MoveIt 결과 코드: {res.error_code.val}  (1 = 성공)")
    if res.planned_trajectory.joint_trajectory.points:
        print(f"궤적 점 수: {len(res.planned_trajectory.joint_trajectory.points)}, "
              f"시간: {res.planned_trajectory.joint_trajectory.points[-1].time_from_start.sec}초")

    js = {}
    def cb(m):
        js.update(dict(zip(m.name, m.position)))
    node.create_subscription(JointState, "/joint_states", cb, 10)
    for _ in range(20):
        rclpy.spin_once(node, timeout_sec=0.1)
    if js:
        print("실제 관절 [deg]:", [round(math.degrees(js[j]), 2) for j in JOINTS])
    rclpy.shutdown()
    return 0 if res.error_code.val == 1 else 2


if __name__ == "__main__":
    sys.exit(main())
