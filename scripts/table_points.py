#!/usr/bin/env python3
"""테이블 위치 측정: 손가락 끝을 테이블에 대고 플랜지(link_6) 자세를 기록 → 평면 맞춤.

읽기 전용 DRFL 연결(tools/flange_pose)을 사용 → 제어권 불필요, TP 수동 모드/직접 교시 중에도 사용 가능.

  python3 table_points.py add p1        # 현재 자세를 p1 으로 기록 (같은 이름이면 덮어씀)
  python3 table_points.py list          # 기록 목록
  python3 table_points.py del p1        # 삭제
  python3 table_points.py fit 180       # 툴 길이 180 mm (플랜지 면 → 닫힌 손가락 끝)로 테이블 평면 계산

저장: ~/doosan_ros2_ws/table_points.json  (플랜지 위치 [mm] + 툴 Z축 방향)
"""
import json
import math
import os
import sys
import time

FILE = os.path.expanduser("~/doosan_ros2_ws/table_points.json")


def load():
    try:
        with open(FILE) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"points": {}}


def save(d):
    tmp = FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump(d, f, indent=2, ensure_ascii=False)
    os.replace(tmp, FILE)


def _unused_quat_to_R(x, y, z, w):
    return [[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]]


def read_flange():
    """읽기 전용 DRFL 도구(tools/flange_pose)로 플랜지 위치와 툴 Z축을 읽음. 제어권을 가져오지 않으므로
    TP 수동 조작(직접 교시) 중에도 사용 가능. ROS 드라이버가 없어도 됨."""
    import subprocess
    exe = os.path.expanduser("~/doosan_ros2_ws/tools/flange_pose")
    try:
        out = subprocess.run([exe, "192.168.137.100"], capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired:
        raise SystemExit("컨트롤러 응답 없음 (네트워크 확인)")
    if out.returncode != 0:
        raise SystemExit("플랜지 자세 읽기 실패: " + out.stderr.strip())
    j = json.loads(out.stdout.strip().splitlines()[-1])
    return j["flange_mm"], j["tool_z"]


def fit_plane(pts):
    """z = a x + b y + c 최소제곱 (점 3개 이상)."""
    n = len(pts)
    sx = sum(p[0] for p in pts); sy = sum(p[1] for p in pts); sz = sum(p[2] for p in pts)
    sxx = sum(p[0] ** 2 for p in pts); syy = sum(p[1] ** 2 for p in pts); sxy = sum(p[0] * p[1] for p in pts)
    sxz = sum(p[0] * p[2] for p in pts); syz = sum(p[1] * p[2] for p in pts)
    A = [[sxx, sxy, sx], [sxy, syy, sy], [sx, sy, n]]
    b = [sxz, syz, sz]

    def det(m):
        return (m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
                + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]))
    D = det(A)
    if abs(D) < 1e-9:
        return None
    sol = []
    for i in range(3):
        M = [row[:] for row in A]
        for r in range(3):
            M[r][i] = b[r]
        sol.append(det(M) / D)
    return sol


def main():
    if len(sys.argv) < 2:
        print(__doc__); return 1
    cmd = sys.argv[1]
    d = load()
    pts = d["points"]
    if cmd == "add":
        name = sys.argv[2] if len(sys.argv) > 2 else f"p{len(pts) + 1}"
        pos, zdir = read_flange()
        pts[name] = {"flange_mm": [round(v, 2) for v in pos], "tool_z": [round(v, 4) for v in zdir],
                     "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        save(d)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -zdir[2]))))
        print(f"기록 '{name}': 플랜지 X {pos[0]:.1f}  Y {pos[1]:.1f}  Z {pos[2]:.1f} mm   "
              f"(툴이 수직 아래에서 {tilt:.1f}° 기울어짐)  — 총 {len(pts)}점")
        if tilt > 15:
            print("  ⚠ 툴이 많이 기울어 있습니다. 손가락 끝 위치 계산 오차가 커질 수 있으니 수직에 가깝게 대 주세요.")
    elif cmd == "list":
        for k, p in pts.items():
            f = p["flange_mm"]
            print(f"{k:6s} 플랜지 [{f[0]:8.1f}, {f[1]:8.1f}, {f[2]:7.1f}] mm  툴Z {p['tool_z']}  {p['time']}")
        print(f"총 {len(pts)}점  ({FILE})")
    elif cmd == "del":
        pts.pop(sys.argv[2], None); save(d); print(f"삭제: {sys.argv[2]}")
    elif cmd == "fit":
        if len(sys.argv) < 3:
            print("툴 길이 [mm] 를 주세요: fit 180"); return 1
        L = float(sys.argv[2])
        tips = []
        for k, p in pts.items():
            f, z = p["flange_mm"], p["tool_z"]
            tips.append([f[i] + z[i] * L for i in range(3)])
            print(f"{k:6s} 손가락 끝 [{tips[-1][0]:8.1f}, {tips[-1][1]:8.1f}, {tips[-1][2]:7.1f}] mm")
        if len(tips) < 3:
            print("평면을 맞추려면 3점 이상 필요합니다."); return 1
        a, b, c = fit_plane(tips)
        res = [t[2] - (a * t[0] + b * t[1] + c) for t in tips]
        slope = math.degrees(math.atan(math.hypot(a, b)))
        xs = [t[0] for t in tips]; ys = [t[1] for t in tips]
        zc = a * (sum(xs) / len(xs)) + b * (sum(ys) / len(ys)) + c
        print(f"\n테이블 면: z = {a:.5f}·x + {b:.5f}·y + {c:.1f}  (기울기 {slope:.2f}°)")
        print(f"측정점 중심에서 테이블 높이 Z = {zc:.1f} mm (로봇 베이스 기준)")
        print(f"점별 오차 [mm]: {[round(r, 1) for r in res]}  (최대 {max(abs(r) for r in res):.1f})")
        print(f"측정 범위 X {min(xs):.0f} ~ {max(xs):.0f}, Y {min(ys):.0f} ~ {max(ys):.0f} mm")
        d["fit"] = {"tool_length_mm": L, "plane": [a, b, c], "z_center_mm": zc, "slope_deg": slope,
                    "x_range": [min(xs), max(xs)], "y_range": [min(ys), max(ys)],
                    "max_residual_mm": max(abs(r) for r in res), "time": time.strftime("%Y-%m-%d %H:%M:%S")}
        save(d)
    else:
        print(__doc__); return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
