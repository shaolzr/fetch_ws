#! /usr/bin/env python3
"""
Add floor & table to MoveIt PlanningScene
"""

# 导入ROS Python客户端库
import rospy
# 导入MoveIt规划场景接口
from moveit_python import PlanningSceneInterface
# 导入MoveIt规划场景消息类型
from moveit_msgs.msg import PlanningScene, ObjectColor
# 导入ROS颜色消息类型
from std_msgs.msg import ColorRGBA

# ------------ 桌子参数：按实际改 ------------
# 定义桌子的尺寸和位置参数
TABLE = dict(size=(0.50, 1.0, 0.72),   # x, y, z (m) - 桌子的长宽高
             pos =(1.00, 0.0, 0.72/2))  # center in base_link - 桌子中心点在base_link坐标系中的位置

# 定义地面的尺寸和位置参数
FLOOR = dict(size=(2.0, 2.0, 0.01),    # x, y, z (m) - 地面的长宽高
             pos =(0.0, 0.0, 0.01/2))   # center in base_link - 地面中心点在base_link坐标系中的位置

def wait_sim():
    """等待ROS时间初始化完成"""
    while rospy.Time().now().to_sec() == 0:
        rospy.sleep(0.1)

def main():
    """主函数"""
    # 初始化ROS节点
    rospy.init_node('lab26_obstacles')
    # 等待时间初始化
    wait_sim()

    # 创建规划场景接口对象，使用base_link作为参考坐标系
    ps = PlanningSceneInterface('base_link')
    # 清除所有碰撞对象
    ps.clear()
    # 移除已存在的地面和桌子对象
    ps.removeCollisionObject('floor')
    ps.removeCollisionObject('table')

    # 添加地面和桌子到规划场景
    ps.addBox('floor', *FLOOR['size'], *FLOOR['pos'])  # 添加地面
    ps.addBox('table', *TABLE['size'], *TABLE['pos'])  # 添加桌子
    rospy.loginfo("Floor & table added.")  # 打印添加成功的日志信息

    # ---- 上色（灰色） -------------------------------------------------
    # 创建规划场景发布者
    pub = rospy.Publisher('/planning_scene', PlanningScene, queue_size=1, latch=True)
    # 等待发布者初始化
    rospy.sleep(0.5)

    # 定义灰色
    grey = ColorRGBA(0.5, 0.5, 0.5, 1.0)  # RGBA值：(0.5, 0.5, 0.5, 1.0)表示灰色
    # 创建规划场景消息，设置对象颜色
    pc_msg = PlanningScene(is_diff=True,  # 设置为差分更新模式
                           object_colors=[
                               ObjectColor(id='floor', color=grey),  # 设置地面颜色
                               ObjectColor(id='table', color=grey)]) # 设置桌子颜色
    # 发布颜色更新消息
    pub.publish(pc_msg)
    # 打印颜色应用成功的日志信息
    rospy.loginfo("Colors applied to objects.")

# 当脚本直接运行时执行main函数
if __name__ == '__main__':
    main()
