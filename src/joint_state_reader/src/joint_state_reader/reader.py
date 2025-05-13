#!/usr/bin/env python
# 订阅joint state
import rospy # 导入 ROS Python 库
from sensor_msgs.msg import JointState # 导入 JointState 消息类型（用于关节状态）

class JointStateReader(object): # 定义一个 JointStateReader 类
    def __init__(self):
        # 初始化：创建一个空字典，用于保存关节的最新状态
        self._joint_states = {} # 创建一个 ROS 订阅者，订阅 '/joint_states' 话题
        rospy.Subscriber('/joint_states', JointState, self._callback) # 每当有 JointState 消息到达，就调用 self._callback 处理

    def _callback(self, msg):
        # 回调函数：每当收到 JointState 消息时执行
        # msg.name 是关节名字列表，例如 ['shoulder_pan_joint', 'elbow_flex_joint']
        # msg.position 是对应关节角度值，例如 [1.32, 1.71]
        for name, position in zip(msg.name, msg.position):
            # 把每个关节名字和角度存入字典，保存最新值
            # 例如：self._joint_states['shoulder_pan_joint'] = 1.32
            self._joint_states[name] = position

    def get_joint(self, name):
        # 查询某个关节的当前角度
        # 如果字典里有，返回角度值；如果没有，返回 None
        return self._joint_states.get(name)

    def get_joints(self, names):
        # 查询多个关节的当前角度，按输入名字列表顺序返回对应角度列表
        # 例如：输入 ['shoulder_pan_joint', 'elbow_flex_joint']
        # 返回 [1.32, 1.71]
        return [self._joint_states.get(n) for n in names]

"""
[Gazebo / 机器人驱动]  → (发布 JointState 消息到 /joint_states) → 
[JointStateReader 订阅者] → (接收 msg + 更新字典) → 
[其他模块调用 reader.get_joint / get_joints 查询]
"""