#!/usr/bin/env python3
"""현재 관절에서 한 관절만 상대 이동 (MoveIt 계획+실행).
사용: python3 moveit_rel_joint.py 6 5 0.1    → J6 +5°, 속도/가속 10%
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState

JOINTS = [f"joint_{i}" for i in range(1, 7)]


def read_joints(node, topic):
    got = {}
    sub = node.create_subscription(JointState, topic, lambda m: got.update(dict(zip(m.name, m.position))), 10)
    t0 = time.time()
    while len(got) < 6 and time.time() - t0 < 3:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return [got[j] for j in JOINTS] if len(got) >= 6 else None


def main():
    j = int(sys.argv[1]) if len(sys.argv) > 1 else 6
    d = float(sys.argv[2]) if len(sys.argv) > 2 else 5.0
    scale = float(sys.argv[3]) if len(sys.argv) > 3 else 0.1
    if not (1 <= j <= 6 and abs(d) <= 30 and 0 < scale <= 0.5):
        print("범위: 관절 1~6, 이동 ±30° 이하, 속도 0~0.5"); return 1
    rclpy.init()
    node = Node("moveit_rel_joint")
    cur = read_joints(node, "/joint_states")
    if cur is None:
        print("joint_states 없음"); return 1
    tgt = list(cur)
    tgt[j - 1] += math.radians(d)
    print("현재 [deg]:", [round(math.degrees(v), 3) for v in cur])
    print("목표 [deg]:", [round(math.degrees(v), 3) for v in tgt], f"(J{j} {d:+}°, 속도/가속 {scale:.0%})")

    cli = ActionClient(node, MoveGroup, "/move_action")
    if not cli.wait_for_server(timeout_sec=10):
        print("move_action 서버 없음"); return 1
    goal = MoveGroup.Goal()
    r = goal.request
    r.group_name = "manipulator"
    r.num_planning_attempts = 5
    r.allowed_planning_time = 5.0
    r.max_velocity_scaling_factor = scale
    r.max_acceleration_scaling_factor = scale
    c = Constraints()
    for name, v in zip(JOINTS, tgt):
        c.joint_constraints.append(JointConstraint(joint_name=name, position=v, tolerance_above=1e-3,
                                                   tolerance_below=1e-3, weight=1.0))
    r.goal_constraints.append(c)
    goal.planning_options.plan_only = False
    f = cli.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, f)
    gh = f.result()
    if not gh.accepted:
        print("목표 거부됨"); return 1
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=120)
    res = rf.result().result
    pts = res.planned_trajectory.joint_trajectory.points
    print(f"MoveIt 결과 코드: {res.error_code.val} (1 = 성공), 궤적 {len(pts)}점"
          + (f", {pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9:.2f}초" if pts else ""))
    time.sleep(1.0)
    fin = read_joints(node, "/joint_states")
    if fin:
        print("최종 [deg]:", [round(math.degrees(v), 3) for v in fin])
        print("목표 오차 [deg]:", [round(math.degrees(a - b), 3) for a, b in zip(fin, tgt)])
    rclpy.shutdown()
    return 0 if res.error_code.val == 1 else 2


if __name__ == "__main__":
    sys.exit(main())
