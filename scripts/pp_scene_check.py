#!/usr/bin/env python3
"""MoveIt 장면(테이블, 그리퍼, 고정 물체, 잡은 직육면체) 구성 + 티칭 Pick & Place 경로 검증.

로봇을 움직이지 않습니다 (계획만). MoveIt이 실행 중이어야 합니다 (가상 모드 권장):
  ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1
사용:
  python3 pp_scene_check.py                 # 장면 적용 + 검증
  python3 pp_scene_check.py --scene-only    # 장면만 적용 (RViz에서 보기)
설정: pp_scene.yaml
"""
import json
import math
import os
import sys

import rclpy
import yaml
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (AttachedCollisionObject, CollisionObject, Constraints, JointConstraint, PlanningScene,
                             RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetCartesianPath, GetPositionFK, GetPositionIK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

HERE = os.path.dirname(os.path.abspath(__file__))
JOINTS = [f"joint_{i}" for i in range(1, 7)]
GROUP, TIP, BASE = "manipulator", "link_6", "base_link"


# ---------------- math ----------------
def q_to_R(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def offset_along_tool_z(pose, d_m):
    """pose 를 툴 Z 방향으로 d_m [m] 이동한 새 Pose (d<0 → 툴 반대 = 아래를 볼 때 위쪽)."""
    R = q_to_R(pose.orientation)
    p = Pose()
    p.position.x = pose.position.x + R[0][2] * d_m
    p.position.y = pose.position.y + R[1][2] * d_m
    p.position.z = pose.position.z + R[2][2] * d_m
    p.orientation = pose.orientation
    return p


def box(size_m, center_m, frame):
    s = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[float(v) for v in size_m])
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in center_m)
    p.orientation.w = 1.0
    return s, p


class Checker(Node):
    def __init__(self, cfg):
        super().__init__("pp_scene_check")
        self.cfg = cfg
        self.cli = {n: self.create_client(t, n) for n, t in [
            ("/apply_planning_scene", ApplyPlanningScene), ("/check_state_validity", GetStateValidity),
            ("/compute_fk", GetPositionFK), ("/compute_ik", GetPositionIK),
            ("/compute_cartesian_path", GetCartesianPath)]}
        self.move = ActionClient(self, MoveGroup, "/move_action")
        for n, c in self.cli.items():
            if not c.wait_for_service(timeout_sec=10):
                raise SystemExit(f"서비스 없음: {n} (MoveIt 실행 확인)")
        self.move.wait_for_server(timeout_sec=10)
        self.problems = []

    def call(self, name, req):
        f = self.cli[name].call_async(req)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30)
        return f.result()

    # ---------------- scene ----------------
    def gripper_object(self, workpiece=None):
        g = self.cfg["gripper"]
        bx, by, bz = [v / 1000 for v in g["body"]]
        fx, fy, fz = [v / 1000 for v in g["finger_zone"]]
        co = CollisionObject(id="gripper", operation=CollisionObject.ADD)
        co.header.frame_id = TIP
        for size, center in [((bx, by, bz), (0, 0, bz / 2)), ((fx, fy, fz), (0, 0, bz + fz / 2))]:
            s, p = box(size, center, TIP)
            co.primitives.append(s)
            co.primitive_poses.append(p)
        if workpiece:
            (wx, wy, wh), z_bottom = workpiece  # [m], bottom position along tool z [m]
            s, p = box((wx, wy, wh), (0, 0, z_bottom - wh / 2), TIP)
            co.primitives.append(s)
            co.primitive_poses.append(p)
        aco = AttachedCollisionObject(link_name=TIP, object=co, touch_links=["link_6", "link_5"])
        return aco

    def apply(self, world=None, attached=None, remove_attached=False):
        ps = PlanningScene(is_diff=True)
        ps.robot_state.is_diff = True
        if remove_attached:
            r = AttachedCollisionObject(link_name=TIP)
            r.object.id = "gripper"
            r.object.operation = CollisionObject.REMOVE
            ps.robot_state.attached_collision_objects.append(r)
            self.call("/apply_planning_scene", ApplyPlanningScene.Request(scene=ps))
            ps = PlanningScene(is_diff=True)
            ps.robot_state.is_diff = True
        if world:
            ps.world.collision_objects.extend(world)
        if attached:
            ps.robot_state.attached_collision_objects.append(attached)
        ok = self.call("/apply_planning_scene", ApplyPlanningScene.Request(scene=ps)).success
        if not ok:
            raise SystemExit("장면 적용 실패")

    def world_objects(self):
        t = self.cfg["table"]
        objs = []
        # remove previous ones first (same ids are replaced by ADD anyway)
        co = CollisionObject(id="table", operation=CollisionObject.ADD)
        co.header.frame_id = BASE
        sx, sy, th = t["x"][1] - t["x"][0], t["y"][1] - t["y"][0], t["thickness"]
        s, p = box((sx / 1000, sy / 1000, th / 1000),
                   ((t["x"][0] + sx / 2) / 1000, (t["y"][0] + sy / 2) / 1000, (t["top_z"] - th / 2) / 1000), BASE)
        co.primitives.append(s)
        co.primitive_poses.append(p)
        objs.append(co)
        for o in self.cfg.get("fixed_objects") or []:
            c = CollisionObject(id=o["name"], operation=CollisionObject.ADD)
            c.header.frame_id = BASE
            sx, sy, h = o["size"]
            s, p = box((sx / 1000, sy / 1000, h / 1000),
                       (o["center"][0] / 1000, o["center"][1] / 1000, (t["top_z"] + h / 2) / 1000), BASE)
            c.primitives.append(s)
            c.primitive_poses.append(p)
            objs.append(c)
        return objs

    # ---------------- queries ----------------
    def state(self, joints_rad):
        rs = RobotState(is_diff=True)
        rs.joint_state = JointState(name=JOINTS, position=list(joints_rad))
        return rs

    def fk(self, q):
        r = GetPositionFK.Request(fk_link_names=[TIP], robot_state=self.state(q))
        r.header.frame_id = BASE
        return self.call("/compute_fk", r).pose_stamped[0].pose

    def ik(self, pose, seed):
        r = GetPositionIK.Request()
        r.ik_request.group_name = GROUP
        r.ik_request.robot_state = self.state(seed)
        r.ik_request.avoid_collisions = False
        r.ik_request.ik_link_name = TIP
        r.ik_request.pose_stamped = PoseStamped(pose=pose)
        r.ik_request.pose_stamped.header.frame_id = BASE
        r.ik_request.timeout.sec = 1
        res = self.call("/compute_ik", r)
        if res.error_code.val != 1:
            return None
        js = dict(zip(res.solution.joint_state.name, res.solution.joint_state.position))
        return [js[j] for j in JOINTS]

    def valid(self, q, label):
        r = GetStateValidity.Request(robot_state=self.state(q), group_name=GROUP)
        res = self.call("/check_state_validity", r)
        if res.valid:
            print(f"  ✅ {label}: 충돌 없음")
        else:
            pairs = sorted({f"{c.contact_body_1} ↔ {c.contact_body_2}" for c in res.contacts})
            print(f"  ❌ {label}: 충돌 {', '.join(pairs) if pairs else '(제약 위반)'}")
            self.problems.append(f"{label}: 충돌 {', '.join(pairs)}")
        return res.valid

    def cartesian(self, q_start, pose_goal, label):
        r = GetCartesianPath.Request(start_state=self.state(q_start), group_name=GROUP, link_name=TIP,
                                     waypoints=[pose_goal], max_step=0.005, jump_threshold=0.0, avoid_collisions=True)
        r.header.frame_id = BASE
        res = self.call("/compute_cartesian_path", r)
        ok = res.fraction > 0.999
        print(f"  {'✅' if ok else '❌'} {label}: 직선 경로 {res.fraction * 100:.0f}% 계획됨")
        if not ok:
            self.problems.append(f"{label}: 직선 경로 {res.fraction * 100:.0f}%만 가능 (충돌 또는 IK 불가)")
            return None
        js = res.solution.joint_trajectory
        last = dict(zip(js.joint_names, js.points[-1].positions)) if js.points else None
        return [last[j] for j in JOINTS] if last else list(q_start)

    def joint_plan(self, q_start, q_goal, label):
        g = MoveGroup.Goal()
        g.request.group_name = GROUP
        g.request.start_state = self.state(q_start)
        g.request.num_planning_attempts = 5
        g.request.allowed_planning_time = 5.0
        g.request.max_velocity_scaling_factor = 0.2
        g.request.max_acceleration_scaling_factor = 0.2
        c = Constraints()
        for n, v in zip(JOINTS, q_goal):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=v, tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        g.request.goal_constraints.append(c)
        g.planning_options.plan_only = True
        f = self.move.send_goal_async(g)
        rclpy.spin_until_future_complete(self, f, timeout_sec=30)
        rf = f.result().get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60)
        res = rf.result().result
        ok = res.error_code.val == 1
        n = len(res.planned_trajectory.joint_trajectory.points)
        print(f"  {'✅' if ok else '❌'} {label}: 관절 경로 계획 {'성공' if ok else '실패'}"
              + (f" ({n}점)" if ok else f" (코드 {res.error_code.val})"))
        if not ok:
            self.problems.append(f"{label}: 경로 계획 실패 (코드 {res.error_code.val})")
        return ok


def table_surface(x_mm, y_mm):
    """측정 평면 (table_points.json fit). 없으면 None."""
    try:
        with open(os.path.expanduser("~/doosan_ros2_ws/table_points.json")) as f:
            fit = json.load(f)["fit"]
        a, b, c = fit["plane"]
        return a * x_mm + b * y_mm + c + fit.get("reference_offset_mm", 0.0)  # 책상 접촉점 보정
    except (OSError, KeyError, ValueError):
        return None


def main():
    cfg = yaml.safe_load(open(os.path.join(HERE, "pp_scene.yaml")))
    with open(os.path.expanduser(cfg["points_file"])) as f:
        pts = json.load(f)["points"]
    for k in ("home", "pick", "place"):
        if k not in pts:
            raise SystemExit(f"티칭 위치 '{k}' 없음 ({cfg['points_file']})")
    q = {k: [math.radians(v) for v in pts[k]["joints"]] for k in ("home", "pick", "place")}

    rclpy.init()
    n = Checker(cfg)
    n.apply(world=n.world_objects(), attached=n.gripper_object(), remove_attached=True)
    t = cfg["table"]
    print(f"장면 적용: 테이블(윗면 Z {t['top_z']} mm, X {t['x']}, Y {t['y']}), 그리퍼(본체 {cfg['gripper']['body']} + "
          f"손가락 영역 {cfg['gripper']['finger_zone']} mm), 고정 물체 {len(cfg.get('fixed_objects') or [])}개")
    if "--scene-only" in sys.argv:
        rclpy.shutdown(); return 0

    L = cfg["gripper"]["tool_length"]
    appr = cfg["approach"] / 1000
    pose = {k: n.fk(q[k]) for k in q}

    print("\n[1] 티칭 위치: 손가락 끝 높이")
    tip = {}
    for k in ("home", "pick", "place"):
        tp = offset_along_tool_z(pose[k], L / 1000)
        x, y, z = tp.position.x * 1000, tp.position.y * 1000, tp.position.z * 1000
        tip[k] = (x, y, z)
        surf = table_surface(x, y)
        print(f"  {k:6s} 손가락 끝 [{x:6.0f}, {y:6.0f}, {z:6.1f}] mm   테이블 박스 위 {z - t['top_z']:6.1f} mm"
              + (f" / 측정면 위 {z - surf:6.1f} mm" if surf is not None else ""))
        if z - t["top_z"] < 5:
            n.problems.append(f"{k}: 손가락 끝이 테이블 박스에 너무 가까움 ({z - t['top_z']:.1f} mm)")

    def above(pz, z_mm):
        p = Pose()
        p.position.x, p.position.y, p.position.z = pz.position.x, pz.position.y, z_mm / 1000
        p.orientation = pz.orientation
        return p

    tz = cfg["transit_z"]
    print(f"\n[2] 경유점: 접근점 (바로 위 {cfg['approach']:.0f} mm, 책상에 수직), 이동 높이 (플랜지 Z {tz:.0f} mm)")
    for k in ("pick", "place"):
        pose[k + "_app"] = above(pose[k], pose[k].position.z * 1000 + cfg["approach"])  # 바로 위 (베이스 Z, 책상에 수직)
        pose[k + "_top"] = above(pose[k], tz)
        if pose[k + "_app"].position.z * 1000 > tz:
            n.problems.append(f"{k} 접근점(Z {pose[k + '_app'].position.z * 1000:.0f})이 이동 높이({tz:.0f})보다 높음")
        for s_ in ("_app", "_top"):
            qa = n.ik(pose[k + s_], q[k])
            if qa is None:
                print(f"  ❌ {k}{s_}: IK 해 없음"); n.problems.append(f"{k}{s_} IK 실패"); continue
            dj = max(abs(math.degrees(x - y)) for x, y in zip(qa, q[k]))
            print(f"  ✅ {k}{s_}: IK 성공, 티칭 자세에서 최대 관절 변화 {dj:.1f}°")
            q[k + s_] = qa
    if any(k not in q for k in ("pick_app", "pick_top", "place_app", "place_top")):
        print("  경유점 IK 실패 → 경로 검사 중단")
        return finish(n)

    print("\n[3] 자세별 충돌 검사 (빈손)")
    for k in ("home", "pick_top", "pick_app", "pick", "place_top", "place_app", "place"):
        n.valid(q[k], k)

    print("\n[4] 빈손 구간 (가는 길 / 돌아오는 길)")
    n.joint_plan(q["home"], q["pick_top"], "① home → pick 위 이동 높이 (관절 경로)")
    j = n.cartesian(q["pick_top"], pose["pick_app"], "② pick 위 → 접근점 수직 하강")
    if j: n.cartesian(j, pose["pick"], "③ 접근점 → pick 하강 (느리게)")
    j = n.cartesian(q["place"], pose["place_app"], "⑨ place → 접근점 상승 (놓은 뒤)")
    if j: j = n.cartesian(j, pose["place_top"], "⑩ 접근점 → 이동 높이 상승")
    if j: n.joint_plan(j, q["home"], "⑪ place 위 → home (관절 경로)")

    w = cfg.get("workpiece") or {}
    fx, fy = w.get("footprint", [20.0, 20.0])
    desk_pick = table_surface(*tip["pick"][:2])
    desk_place = table_surface(*tip["place"][:2])
    g = tip["pick"][2] - desk_pick              # 손가락 끝이 물체 밑면(=책상) 위 몇 mm
    finger = cfg["gripper"]["finger_zone"][2]
    print(f"\n[5] 물체를 잡은 구간 (pick 에서 손가락 끝 = 물체 밑면 위 {g:.1f} mm)")
    ok_heights = []
    for h in w.get("heights", []):
        depth = h - g
        if depth < 5:
            print(f"  ⏸ {h:.0f} mm 물체: 손가락이 윗면에서 {depth:.1f} mm → 잡을 수 없음/너무 얕음 (이 pick 높이와 안 맞음)")
        elif depth > finger + 3:
            print(f"  ⚠ {h:.0f} mm 물체: 윗면에서 {depth:.1f} mm 아래를 잡음 → 손가락 길이(~{finger:.0f} mm)보다 깊어 본체가 닿을 수 있음")
            ok_heights.append(h)
        else:
            print(f"  ✅ {h:.0f} mm 물체: 윗면에서 {depth:.1f} mm 아래를 잡음")
            ok_heights.append(h)
    if not ok_heights:
        n.problems.append("pick 높이로 잡을 수 있는 물체가 없음")
    t_top = cfg["table"]["top_z"]
    for h in ok_heights:
        print(f"  --- {h:.0f} mm 물체 ---")
        # 모델의 물체 밑면을 테이블 박스(평평, {t_top} mm) 위로 띄움: pick 에서 박스와 겹치지 않게
        trim = max(3.0, t_top - desk_pick + 3.0)
        bottom_along_tool = (L + g - trim) / 1000
        n.apply(attached=n.gripper_object(((fx / 1000, fy / 1000, (h - trim) / 1000), bottom_along_tool)),
                remove_attached=True)
        for k in ("pick_app", "pick_top", "place_top", "place_app"):
            n.valid(q[k], f"{k} (물체 잡음)")
        j = n.cartesian(q["pick"], pose["pick_app"], "④ pick → 접근점 상승")
        if j: j = n.cartesian(j, pose["pick_top"], "⑤ 접근점 → 이동 높이 상승")
        if j: j = n.cartesian(j, pose["place_top"], "⑥ 이동 높이에서 place 위로 수평 이동")
        if j: j = n.cartesian(j, pose["place_app"], "⑦ place 위 → 접근점 수직 하강")
        bottom_pl = tip["place"][2] - g
        gap = bottom_pl - desk_place
        print(f"  place 에서 그리퍼를 열 때 물체 밑면: 책상 위 {gap:.1f} mm 에서 떨어짐")
        if gap < 0:
            n.problems.append(f"{h:.0f} mm 물체: place 에서 밑면이 책상 아래 {-gap:.1f} mm → 충돌")
        elif gap > 10:
            n.problems.append(f"{h:.0f} mm 물체: place 에서 {gap:.1f} mm 높이에서 떨어짐 → 넘어질 수 있음 (권장 3~5 mm)")
        if j: n.cartesian(j, pose["place"], "⑧ 접근점 → place 하강 (느리게)")
    n.apply(attached=n.gripper_object(), remove_attached=True)  # 장면을 빈손 상태로 되돌림
    return finish(n)


def finish(n):
    print("\n==== 결과 ====")
    if n.problems:
        for p in n.problems:
            print("  ⚠", p)
    else:
        print("  ✅ 문제 없음")
    rclpy.shutdown()
    return 1 if n.problems else 0


if __name__ == "__main__":
    sys.exit(main())
