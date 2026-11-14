#!/usr/bin/env python3
"""RViz markers: desk, gripper (colored by /gripper_state), held workpiece, taught home/pick/place and the
15-step Pick & Place path of the DRFL GUI (vertical approach + transit height).

Files are re-read every 2 s, so points re-taught in the GUI show up without restarting.
  scene:  ~/doosan_ros2_ws/scripts/pp_scene.yaml   (table, gripper, workpiece, approach, transit_z)
  points: points_file in that yaml (~/m0609_points_<serial>.json)
"""
import json
import math
import os

import rclpy
import yaml
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_msgs.msg import ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


def zyz_R(a, b, c):
    a, b, c = (math.radians(v) for v in (a, b, c))
    ca, sa, cb, sb, cc, sc = math.cos(a), math.sin(a), math.cos(b), math.sin(b), math.cos(c), math.sin(c)
    return [[ca * cb * cc - sa * sc, -ca * cb * sc - sa * cc, ca * sb],
            [sa * cb * cc + ca * sc, -sa * cb * sc + ca * cc, sa * sb],
            [-sb * cc, sb * sc, cb]]


def col(r, g, b, a=1.0):
    return ColorRGBA(r=float(r), g=float(g), b=float(b), a=float(a))


def pt(x, y, z):
    return Point(x=float(x), y=float(y), z=float(z))


class SceneMarkers(Node):
    def __init__(self):
        super().__init__("scene_markers")
        self.scene_file = os.path.expanduser(self.declare_parameter(
            "scene_file", "~/doosan_ros2_ws/scripts/pp_scene.yaml").value)
        self.pub = self.create_publisher(MarkerArray, "pp_markers", 10)
        self.grip = "unknown"
        self.create_subscription(String, "gripper_state", self.on_grip, 10)
        self.cfg, self.pts = {}, {}
        self.reload()
        self.create_timer(2.0, self.reload)
        self.create_timer(0.1, self.publish)

    def on_grip(self, m):
        self.grip = m.data

    def reload(self):
        try:
            with open(self.scene_file) as f:
                self.cfg = yaml.safe_load(f) or {}
            pf = os.path.expanduser(self.cfg.get("points_file", ""))
            if not os.path.exists(pf):  # not taught on this PC yet -> example points shipped in the repo
                pf = os.path.join(os.path.dirname(self.scene_file), "example_points", os.path.basename(pf))
            with open(pf) as f:
                self.pts = json.load(f).get("points", {})
        except (OSError, ValueError, yaml.YAMLError) as e:
            self.get_logger().warn(f"reload: {e}", throttle_duration_sec=10.0)

    def mk(self, mid, typ, frame="base_link", ns="scene"):
        m = Marker()
        m.header.frame_id = frame
        # stamp 0 = "latest TF": a 'now' stamp makes RViz wait for a future link_6 transform and drop the marker
        m.frame_locked = frame != "base_link"
        m.ns, m.id, m.type, m.action = ns, mid, typ, Marker.ADD
        m.pose.orientation.w = 1.0
        return m

    def publish(self):
        arr = MarkerArray()
        c = self.cfg
        t = c.get("table")
        if t:
            m = self.mk(0, Marker.CUBE)
            sx, sy = t["x"][1] - t["x"][0], t["y"][1] - t["y"][0]
            m.pose.position = pt((t["x"][0] + sx / 2) / 1000, (t["y"][0] + sy / 2) / 1000,
                                 (t["top_z"] - t["thickness"] / 2) / 1000)
            m.scale.x, m.scale.y, m.scale.z = sx / 1000, sy / 1000, t["thickness"] / 1000
            m.color = col(0.55, 0.45, 0.35, 0.6)
            arr.markers.append(m)
        g = c.get("gripper")
        L = (g or {}).get("tool_length", 140.0)
        if g:
            bx, by, bz = g["body"]
            m = self.mk(1, Marker.CUBE, "link_6", "gripper")
            m.pose.position = pt(0, 0, bz / 2000)
            m.scale.x, m.scale.y, m.scale.z = bx / 1000, by / 1000, bz / 1000
            m.color = col(0.55, 0.58, 0.65, 0.85)
            arr.markers.append(m)
            fx, fy, fz = g["finger_zone"]
            m = self.mk(2, Marker.CUBE, "link_6", "gripper")
            m.pose.position = pt(0, 0, (bz + fz / 2) / 1000)
            m.scale.x, m.scale.y, m.scale.z = fx / 1000, fy / 1000, fz / 1000
            m.color = {"open": col(0.2, 0.8, 0.2), "closed": col(1.0, 0.6, 0.0), "holding": col(0.1, 0.4, 1.0),
                       "error": col(1, 0, 0)}.get(self.grip, col(0.7, 0.7, 0.7))
            arr.markers.append(m)
            m = self.mk(3, Marker.TEXT_VIEW_FACING, "link_6", "gripper")
            m.pose.position = pt(0, 0.08, bz / 2000)
            m.scale.z = 0.03
            m.color = col(1, 1, 1)
            # RViz text markers have no Hangul glyphs -> English labels
            m.text = {"open": "OPEN", "closed": "CLOSED", "holding": "HOLDING", "middle": "MIDDLE",
                      "error": "ERROR"}.get(self.grip, "?")  # single words: RViz renders spaces badly
            arr.markers.append(m)
            w = c.get("workpiece") or {}
            m = self.mk(4, Marker.CUBE, "link_6", "gripper")
            h = (w.get("heights") or [60.0])[0]
            fp = w.get("footprint", [20.0, 20.0])
            grip_below_top = 17.0  # pick at 43 mm above the desk on a 60 mm piece (validated 2026-11-13)
            m.pose.position = pt(0, 0, (L + h / 2 - grip_below_top) / 1000)
            m.scale.x, m.scale.y, m.scale.z = fp[0] / 1000, fp[1] / 1000, h / 1000
            m.color = col(0.9, 0.2, 0.2, 0.9)
            if self.grip != "holding":
                m.action = Marker.DELETE
            arr.markers.append(m)

        # taught points: fingertip spheres + labels, and the P&P path of the fingertip
        appr, tz = c.get("approach", 100.0), c.get("transit_z", 350.0)
        tips = {}
        for i, name in enumerate(("home", "pick", "place")):
            p = self.pts.get(name)
            if not p:
                continue
            x = p["posx"]
            R = zyz_R(*x[3:])
            off = [R[k][2] * L for k in range(3)]
            tip = [x[k] + off[k] for k in range(3)]
            tips[name] = (x, off, tip)
            m = self.mk(10 + i, Marker.SPHERE, ns="points")
            m.pose.position = pt(*(v / 1000 for v in tip))
            m.scale.x = m.scale.y = m.scale.z = 0.015
            m.color = {"home": col(0.6, 0.6, 1.0), "pick": col(0.1, 0.9, 0.1), "place": col(1.0, 0.3, 0.3)}[name]
            arr.markers.append(m)
            m = self.mk(20 + i, Marker.TEXT_VIEW_FACING, ns="points")
            m.pose.position = pt(tip[0] / 1000, tip[1] / 1000, tip[2] / 1000 + 0.03)
            m.scale.z = 0.03
            m.color = col(1, 1, 1)
            m.text = name
            arr.markers.append(m)
        if "pick" in tips and "place" in tips:
            def tip_at(name, flange_z):
                x, off, _ = tips[name]
                return pt((x[0] + off[0]) / 1000, (x[1] + off[1]) / 1000, (flange_z + off[2]) / 1000)
            pk, pl = tips["pick"][0], tips["place"][0]
            path = [tip_at("pick", tz), tip_at("pick", pk[2] + appr), tip_at("pick", pk[2]),
                    tip_at("pick", pk[2] + appr), tip_at("pick", tz), tip_at("place", tz),
                    tip_at("place", pl[2] + appr), tip_at("place", pl[2])]
            m = self.mk(30, Marker.LINE_STRIP, ns="path")
            m.points = path
            m.scale.x = 0.004
            m.color = col(1.0, 0.85, 0.1, 0.9)
            arr.markers.append(m)
            m = self.mk(31, Marker.TEXT_VIEW_FACING, ns="path")
            mid = path[4]
            m.pose.position = pt(mid.x, (mid.y + path[5].y) / 2, mid.z + 0.03)
            m.scale.z = 0.025
            m.color = col(1.0, 0.85, 0.1)
            m.text = f"Z{tz:.0f}"
            arr.markers.append(m)
        self.pub.publish(arr)


def main():
    rclpy.init()
    rclpy.spin(SceneMarkers())


if __name__ == "__main__":
    main()
