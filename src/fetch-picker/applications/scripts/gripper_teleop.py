#!/usr/bin/env python3
# Lab-26 – Gripper & Auto-Pick teleoperation (完整修正版)

import sys
import copy
import rospy
import numpy as np
import tf.transformations as tft
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_commander import roscpp_initialize
import robot_api      # 需确保你的 Arm() / Gripper() 封装可用

# ── Mesh 路径 ───────────────────────────────────────
GRIPPER_MESH  = 'package://fetch_description/meshes/gripper_link.dae'
L_FINGER_MESH = 'package://fetch_description/meshes/l_gripper_finger_link.STL'
R_FINGER_MESH = 'package://fetch_description/meshes/r_gripper_finger_link.STL'

# ── 实用函数 ───────────────────────────────────────
def q_about(axis):
    ang = np.pi / 2
    if axis == 'x':
        q = tft.quaternion_about_axis(ang, (1, 0, 0))
    elif axis == 'y':
        q = tft.quaternion_about_axis(ang, (0, 1, 0))
    elif axis == 'z':
        q = tft.quaternion_about_axis(ang, (0, 0, 1))
    else:
        q = (0, 0, 0, 1)
    return Quaternion(*q)

def make_6dof():
    ctrls = []
    for ax in 'xyz':
        for mode, name in [(InteractiveMarkerControl.MOVE_AXIS, 'move'),
                           (InteractiveMarkerControl.ROTATE_AXIS, 'rot')]:
            c = InteractiveMarkerControl()
            c.name = f'{name}_{ax}'
            c.interaction_mode = mode
            q = q_about(ax)
            c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = q.x, q.y, q.z, q.w
            ctrls.append(c)
    return ctrls

def gripper_meshes(rgb=(0, 1, 0)):
    """掌心 + 两指尖三块网格，已校正偏移"""
    def mesh(res, dx=0, dy=0, dz=0):
        m = Marker()
        m.type = Marker.MESH_RESOURCE
        m.mesh_resource = res
        m.scale.x = m.scale.y = m.scale.z = 1.0
        m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0
        m.pose.position.x, m.pose.position.y, m.pose.position.z = dx, dy, dz
        return m

    PALM_DX   = 0.166      # wrist_roll_link → gripper_link
    FINGER_DY = 0.018      # 指尖左右 ±1.8 cm
    FINGER_DZ = 0.004      # 指尖略高 4 mm

    return [
        mesh(GRIPPER_MESH, PALM_DX, 0.0,        0.0),        # 掌心
        mesh(L_FINGER_MESH, 0.0,   +FINGER_DY,  FINGER_DZ),  # 左指
        mesh(R_FINGER_MESH, 0.0,   -FINGER_DY,  FINGER_DZ)   # 右指
    ]

# ────────────────────────────────────────────────
class GripperTeleop:
    """绿色抓手：纯 6-DOF Teleop"""
    def __init__(self, arm, gripper, server):
        self.arm, self.gripper, self.server = arm, gripper, server
        self.menu = MenuHandler()
        self.name = 'gripper_marker'

    def start(self):
        self.server.insert(self._marker(), self.cb)
        for title in ['Go to Pose', 'Open Gripper', 'Close Gripper']:
            self.menu.insert(title, callback=self.cb)
        self.menu.apply(self.server, self.name)
        self.server.applyChanges()

    def _marker(self):
        im = InteractiveMarker()
        im.header.frame_id = 'base_link'
        im.name  = self.name
        im.scale = 0.25
        im.pose.position.x, im.pose.position.z = 0.6, 0.8

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = ctrl.MENU
        ctrl.always_visible  = True
        ctrl.markers.extend(gripper_meshes())   # 绿色
        im.controls.append(ctrl)
        im.controls.extend(make_6dof())
        return im

    def cb(self, fb):
        if fb.event_type == fb.MENU_SELECT:
            if fb.menu_entry_id == 1:
                self.arm.move_to_pose(PoseStamped(header=fb.header, pose=fb.pose))
            elif fb.menu_entry_id == 2:
                self.gripper.open()
            elif fb.menu_entry_id == 3:
                self.gripper.close()
        elif fb.event_type == fb.MOUSE_UP:
            ok = self.arm.check_pose(PoseStamped(header=fb.header, pose=fb.pose))
            self._recolor(ok)

    def _recolor(self, ok):
        im = self.server.get(self.name)
        rgb = (0, 1, 0) if ok else (1, 0, 0)
        for m in im.controls[0].markers:
            m.color.r, m.color.g, m.color.b = rgb
        self.server.insert(im, self.cb)
        self.menu.apply(self.server, self.name)
        self.server.applyChanges()

# ────────────────────────────────────────────────
class AutoPickTeleop:
    """蓝色目标方块 + 抓手：右键 Pick 后自动张抓/闭合/抬升"""
    def __init__(self, arm, gripper, server):
        self.arm, self.gripper, self.server = arm, gripper, server
        self.menu = MenuHandler()
        self.name = 'target_marker'

    # ---------- 目标交互标记 ----------
    def _marker(self):
        im = InteractiveMarker()
        im.header.frame_id = 'base_link'
        im.name  = self.name
        im.scale = 0.25
        im.pose.position.x, im.pose.position.z = 0.8, 0.75
        im.pose.orientation.w = 1.0

        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = ctrl.MENU
        ctrl.always_visible  = True

        cube = Marker()
        cube.type = Marker.CUBE
        cube.scale.x = cube.scale.y = cube.scale.z = 0.05
        cube.color.b, cube.color.a = 1.0, 1.0
        cube.pose.position.x = 0.18
        ctrl.markers.append(cube)

        ctrl.markers.extend(gripper_meshes(rgb=(0, 0, 1)))  # 蓝色
        im.controls.append(ctrl)
        im.controls.extend(make_6dof())
        return im

    # ---------- 启动 ----------
    def start(self):
        self.server.insert(self._marker(), self.cb)
        self.id_pick = self.menu.insert('Pick Object',  callback=self.cb)
        self.id_open = self.menu.insert('Open Gripper', callback=self.cb)
        self.menu.apply(self.server, self.name)
        self.server.applyChanges()

    # ---------- 回调 ----------
    def cb(self, fb):
        if fb.event_type == fb.MENU_SELECT:
            if fb.menu_entry_id == self.id_pick:
                self._do_pick(fb)
            elif fb.menu_entry_id == self.id_open:
                self.gripper.open()
        elif fb.event_type == fb.MOUSE_UP:
            ok = self.arm.check_pose(PoseStamped(header=fb.header, pose=fb.pose))
            self._recolor(ok)

    # ---------- 抓取序列 ----------
        # ---------- ③ 抓取动作 ----------
    def _do_pick(self, fb):
        target = PoseStamped(header=fb.header, pose=fb.pose)

        # ① 张开爪子
        rospy.loginfo("Pick: open gripper")
        self.gripper.open(); rospy.sleep(0.2)

        # ② 预抓取 = 目标前 10 cm
        pre_grasp = copy.deepcopy(target)
        pre_grasp.pose.position.x -= 0.10
        rospy.loginfo("Pick: move to pre-grasp (x –10 cm)")
        ok = self.arm.move_to_pose(pre_grasp)
        rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")
        if not ok:
            rospy.logwarn("Planning to pre-grasp failed — aborting pick")
            return

        # ③ 前进到真正抓取位姿
        rospy.loginfo("Pick: move to grasp pose")
        ok = self.arm.move_to_pose(target)
        rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")
        if not ok:
            rospy.logwarn("Planning to grasp pose failed — aborting pick")
            return

        # ④ 闭合爪子
        rospy.loginfo("Pick: close gripper")
        self.gripper.close()
        rospy.sleep(1.0)

        # # ⑤ 抬升 10 cm
        # lift = copy.deepcopy(target)
        # lift.pose.position.z += 0.10
        # rospy.loginfo("Pick: lift 10 cm")
        # ok = self.arm.move_to_pose(lift)
        # rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")


    def _recolor(self, ok):
        im = self.server.get(self.name)
        rgb = (0, 1, 0) if ok else (1, 0, 0)
        for m in im.controls[0].markers:
            if m.type == Marker.CUBE:
                continue
            m.color.r, m.color.g, m.color.b = rgb
        self.server.insert(im, self.cb)
        self.menu.apply(self.server, self.name)
        self.server.applyChanges()

# ────────────────────────────────────────────────
def main():
    roscpp_initialize(sys.argv)
    rospy.init_node('lab26_teleop')

    arm, gripper = robot_api.Arm(), robot_api.Gripper()
    srv_gripper  = InteractiveMarkerServer('gripper_im_server', q_size=2)
    srv_target   = InteractiveMarkerServer('target_im_server',  q_size=2)

    GripperTeleop(arm, gripper, srv_gripper).start()
    AutoPickTeleop(arm, gripper, srv_target).start()
    rospy.spin()

if __name__ == '__main__':
    main()
