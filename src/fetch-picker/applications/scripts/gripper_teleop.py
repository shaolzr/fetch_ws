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
    """根据指定轴生成90度旋转的四元数"""
    ang = np.pi / 2  # 90度角（弧度）
    if axis == 'x':  # 绕X轴旋转
        q = tft.quaternion_about_axis(ang, (1, 0, 0))
    elif axis == 'y':  # 绕Y轴旋转
        q = tft.quaternion_about_axis(ang, (0, 1, 0))
    elif axis == 'z':  # 绕Z轴旋转
        q = tft.quaternion_about_axis(ang, (0, 0, 1))
    else:  # 默认单位四元数
        q = (0, 0, 0, 1)
    return Quaternion(*q)

def make_6dof():
    """创建6自由度交互控制"""
    ctrls = []  # 控制列表
    for ax in 'xyz':  # 遍历三个轴
        for mode, name in [(InteractiveMarkerControl.MOVE_AXIS, 'move'),  # 平移控制
                           (InteractiveMarkerControl.ROTATE_AXIS, 'rot')]:  # 旋转控制
            c = InteractiveMarkerControl()  # 创建控制对象
            c.name = f'{name}_{ax}'  # 设置控制名称
            c.interaction_mode = mode  # 设置交互模式
            q = q_about(ax)  # 获取对应轴的四元数
            c.orientation.x, c.orientation.y, c.orientation.z, c.orientation.w = q.x, q.y, q.z, q.w  # 设置方向
            ctrls.append(c)  # 添加到控制列表
    return ctrls

def gripper_meshes(rgb=(0, 1, 0)):
    """创建夹持器网格标记，包括掌心、左手指和右手指"""
    def mesh(res, dx=0, dy=0, dz=0):
        """创建单个网格标记"""
        m = Marker()  # 创建标记对象
        m.type = Marker.MESH_RESOURCE  # 设置类型为网格资源
        m.mesh_resource = res  # 设置网格资源路径
        m.scale.x = m.scale.y = m.scale.z = 1.0  # 设置缩放比例
        m.color.r, m.color.g, m.color.b, m.color.a = *rgb, 1.0  # 设置颜色
        m.pose.position.x, m.pose.position.y, m.pose.position.z = dx, dy, dz  # 设置位置偏移
        return m

    # 定义偏移常量
    PALM_DX   = 0.166      # 掌心X轴偏移（wrist_roll_link到gripper_link）
    FINGER_DY = 0.018      # 手指Y轴偏移（左右各1.8cm）
    FINGER_DZ = 0.004      # 手指Z轴偏移（上移4mm）

    # 返回三个网格标记
    return [
        mesh(GRIPPER_MESH, PALM_DX, 0.0,        0.0),        # 掌心网格
        mesh(L_FINGER_MESH, 0.0,   +FINGER_DY,  FINGER_DZ),  # 左手指网格
        mesh(R_FINGER_MESH, 0.0,   -FINGER_DY,  FINGER_DZ)   # 右手指网格
    ]

# ────────────────────────────────────────────────
class GripperTeleop:
    """绿色夹持器：纯6自由度遥操作类"""
    def __init__(self, arm, gripper, server):
        """初始化遥操作类"""
        self.arm, self.gripper, self.server = arm, gripper, server  # 保存机械臂、夹持器和服务器引用
        self.menu = MenuHandler()  # 创建菜单处理器
        self.name = 'gripper_marker'  # 设置标记名称

    def start(self):
        """启动遥操作"""
        self.server.insert(self._marker(), self.cb)  # 插入交互标记
        # 添加菜单项
        for title in ['Go to Pose', 'Open Gripper', 'Close Gripper']:
            self.menu.insert(title, callback=self.cb)
        self.menu.apply(self.server, self.name)  # 应用菜单
        self.server.applyChanges()  # 应用更改

    def _marker(self):
        """创建交互标记"""
        im = InteractiveMarker()  # 创建交互标记
        im.header.frame_id = 'base_link'  # 设置参考坐标系
        im.name  = self.name  # 设置名称
        im.scale = 0.25  # 设置缩放比例
        im.pose.position.x, im.pose.position.z = 0.6, 0.8  # 设置初始位置

        # 创建菜单控制
        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = ctrl.MENU  # 设置交互模式为菜单
        ctrl.always_visible  = True  # 设置始终可见
        ctrl.markers.extend(gripper_meshes())   # 添加绿色夹持器网格
        im.controls.append(ctrl)  # 添加控制
        im.controls.extend(make_6dof())  # 添加6自由度控制
        return im

    def cb(self, fb):
        """回调函数处理交互事件"""
        if fb.event_type == fb.MENU_SELECT:  # 菜单选择事件
            if fb.menu_entry_id == 1:  # 移动到指定位姿
                self.arm.move_to_pose(PoseStamped(header=fb.header, pose=fb.pose))
            elif fb.menu_entry_id == 2:  # 打开夹持器
                self.gripper.open()
            elif fb.menu_entry_id == 3:  # 关闭夹持器
                self.gripper.close()
        elif fb.event_type == fb.MOUSE_UP:  # 鼠标释放事件
            ok = self.arm.check_pose(PoseStamped(header=fb.header, pose=fb.pose))  # 检查位姿是否可达
            self._recolor(ok)  # 根据可达性重新着色

    def _recolor(self, ok):
        """根据可达性重新着色"""
        im = self.server.get(self.name)  # 获取标记
        rgb = (0, 1, 0) if ok else (1, 0, 0)  # 可达为绿色，不可达为红色
        for m in im.controls[0].markers:  # 更新所有网格的颜色
            m.color.r, m.color.g, m.color.b = rgb
        self.server.insert(im, self.cb)  # 重新插入标记
        self.menu.apply(self.server, self.name)  # 应用菜单
        self.server.applyChanges()  # 应用更改

# ────────────────────────────────────────────────
class AutoPickTeleop:
    """蓝色目标方块+夹持器：右键Pick后自动张抓/闭合/抬升"""
    def __init__(self, arm, gripper, server):
        """初始化自动抓取类"""
        self.arm, self.gripper, self.server = arm, gripper, server  # 保存机械臂、夹持器和服务器引用
        self.menu = MenuHandler()  # 创建菜单处理器
        self.name = 'target_marker'  # 设置标记名称

    def _marker(self):
        """创建目标交互标记"""
        im = InteractiveMarker()  # 创建交互标记
        im.header.frame_id = 'base_link'  # 设置参考坐标系
        im.name  = self.name  # 设置名称
        im.scale = 0.25  # 设置缩放比例
        im.pose.position.x, im.pose.position.z = 0.8, 0.75  # 设置初始位置
        im.pose.orientation.w = 1.0  # 设置初始方向

        # 创建菜单控制
        ctrl = InteractiveMarkerControl()
        ctrl.interaction_mode = ctrl.MENU  # 设置交互模式为菜单
        ctrl.always_visible  = True  # 设置始终可见

        # 创建目标方块
        cube = Marker()
        cube.type = Marker.CUBE  # 设置类型为立方体
        cube.scale.x = cube.scale.y = cube.scale.z = 0.05  # 设置尺寸
        cube.color.b, cube.color.a = 1.0, 1.0  # 设置蓝色
        cube.pose.position.x = 0.18  # 设置位置
        ctrl.markers.append(cube)  # 添加方块

        ctrl.markers.extend(gripper_meshes(rgb=(0, 0, 1)))  # 添加蓝色夹持器网格
        im.controls.append(ctrl)  # 添加控制
        im.controls.extend(make_6dof())  # 添加6自由度控制
        return im

    def start(self):
        """启动自动抓取"""
        self.server.insert(self._marker(), self.cb)  # 插入交互标记
        self.id_pick = self.menu.insert('Pick Object',  callback=self.cb)  # 添加抓取菜单项
        self.id_open = self.menu.insert('Open Gripper', callback=self.cb)  # 添加打开夹持器菜单项
        self.menu.apply(self.server, self.name)  # 应用菜单
        self.server.applyChanges()  # 应用更改

    def cb(self, fb):
        """回调函数处理交互事件"""
        if fb.event_type == fb.MENU_SELECT:  # 菜单选择事件
            if fb.menu_entry_id == self.id_pick:  # 抓取对象
                self._do_pick(fb)
            elif fb.menu_entry_id == self.id_open:  # 打开夹持器
                self.gripper.open()
        elif fb.event_type == fb.MOUSE_UP:  # 鼠标释放事件
            ok = self.arm.check_pose(PoseStamped(header=fb.header, pose=fb.pose))  # 检查位姿是否可达
            self._recolor(ok)  # 根据可达性重新着色

    def _do_pick(self, fb):
        """执行抓取序列"""
        target = PoseStamped(header=fb.header, pose=fb.pose)  # 创建目标位姿

        # ① 张开夹持器
        rospy.loginfo("Pick: open gripper")
        self.gripper.open(); rospy.sleep(0.2)

        # ② 移动到预抓取位置（目标前10cm）
        pre_grasp = copy.deepcopy(target)
        pre_grasp.pose.position.x -= 0.10
        rospy.loginfo("Pick: move to pre-grasp (x –10 cm)")
        ok = self.arm.move_to_pose(pre_grasp)
        rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")
        if not ok:
            rospy.logwarn("Planning to pre-grasp failed — aborting pick")
            return

        # ③ 移动到抓取位置
        rospy.loginfo("Pick: move to grasp pose")
        ok = self.arm.move_to_pose(target)
        rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")
        if not ok:
            rospy.logwarn("Planning to grasp pose failed — aborting pick")
            return

        # ④ 闭合夹持器
        rospy.loginfo("Pick: close gripper")
        self.gripper.close()
        rospy.sleep(1.0)

        # # ⑤ 抬升10cm（已注释）
        # lift = copy.deepcopy(target)
        # lift.pose.position.z += 0.10
        # rospy.loginfo("Pick: lift 10 cm")
        # ok = self.arm.move_to_pose(lift)
        # rospy.loginfo(f"  ⇢ move_to_pose returned {ok}")

    def _recolor(self, ok):
        """根据可达性重新着色"""
        im = self.server.get(self.name)  # 获取标记
        rgb = (0, 1, 0) if ok else (1, 0, 0)  # 可达为绿色，不可达为红色
        for m in im.controls[0].markers:  # 更新所有网格的颜色
            if m.type == Marker.CUBE:  # 跳过目标方块
                continue
            m.color.r, m.color.g, m.color.b = rgb
        self.server.insert(im, self.cb)  # 重新插入标记
        self.menu.apply(self.server, self.name)  # 应用菜单
        self.server.applyChanges()  # 应用更改

# ────────────────────────────────────────────────
def main():
    """主函数"""
    roscpp_initialize(sys.argv)  # 初始化C++接口
    rospy.init_node('lab26_teleop')  # 初始化ROS节点

    # 创建机械臂和夹持器对象
    arm, gripper = robot_api.Arm(), robot_api.Gripper()
    # 创建交互标记服务器
    srv_gripper  = InteractiveMarkerServer('gripper_im_server', q_size=2)
    srv_target   = InteractiveMarkerServer('target_im_server',  q_size=2)

    # 启动遥操作和自动抓取
    GripperTeleop(arm, gripper, srv_gripper).start()
    AutoPickTeleop(arm, gripper, srv_target).start()
    rospy.spin()  # 进入ROS循环

# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()
