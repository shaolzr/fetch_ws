#!/usr/bin/env python  # 指定使用环境中的 Python 解释器运行脚本
import rospy  # ROS Python 客户端库
import actionlib  # ROS 的 action 客户端库，用于发送/接收导航指令
import pickle  # 用于对象序列化（保存字典到文件）
import os  # 提供操作系统功能，例如路径处理
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped  # ROS 消息类型，用于表示机器人的姿态
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal  # ROS move_base 的 action 消息类型

class MapAnnotator:  # 定义一个类，负责地图标注和导航
    def __init__(self):  # 初始化函数
        rospy.init_node('map_annotator')  # 初始化 ROS 节点，节点名为 map_annotator
        self.pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.pose_callback)  # 订阅 AMCL 的位姿话题
        self.current_pose = None  # 当前位姿，初始为空
        self.poses = {}  # 用来存储保存的位姿（以名字为键）
        self.filename = os.path.expanduser('~/.map_poses.pkl')  # 保存位姿的文件路径，放在用户家目录
        self.load_poses()  # 启动时尝试加载保存的位姿文件
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)  # 创建 move_base action 客户端
        self.client.wait_for_server()  # 等待 move_base action server 可用
        rospy.loginfo("Connected to move_base action server.")  # 日志提示已连接

    def pose_callback(self, msg):  # 接收到 AMCL 位姿话题时的回调函数
        self.current_pose = msg.pose.pose  # 提取出位姿（去掉协方差部分）
    
    def save_pose(self, name):  # 保存当前位姿
        if self.current_pose is None:  # 如果当前没有位姿
            print("No current pose available.")  # 提示没有可保存的位姿
            return
        self.poses[name] = self.current_pose  # 以名字为键保存位姿到字典
        print(f"Saved pose '{name}'.")  # 提示保存成功
    
    def list_poses(self):  # 列出所有保存的位姿
        if not self.poses:  # 如果字典为空
            print("No poses saved.")  # 提示没有保存的位姿
        else:
            print("Poses:")  # 打印标题
            for name in self.poses:  # 遍历字典
                print(f"  {name}")  # 打印每个位姿名字

    def delete_pose(self, name):  # 删除指定名字的位姿
        if name in self.poses:  # 如果名字存在
            del self.poses[name]  # 从字典中删除
            print(f"Deleted pose '{name}'.")  # 提示删除成功
        else:
            print(f"No such pose '{name}'.")  # 提示未找到该名字

    def goto_pose(self, name):  # 导航到指定名字的位姿
        if name not in self.poses:  # 如果字典中没有该名字
            print(f"No such pose '{name}'.")  # 提示错误
            return
        pose = self.poses[name]  # 获取保存的位姿
        goal = MoveBaseGoal()  # 创建导航目标对象
        goal.target_pose.header.frame_id = 'map'  # 设置坐标系为 map
        goal.target_pose.header.stamp = rospy.Time.now()  # 设置时间戳为当前时间
        goal.target_pose.pose = pose  # 设置目标位姿
        self.client.send_goal(goal)  # 发送导航目标
        self.client.wait_for_result()  # 等待导航完成
        result = self.client.get_result()  # 获取导航结果
        print(f"Arrived at pose '{name}'. Result: {result}")  # 打印结果

    def save_to_disk(self):  # 将位姿字典保存到文件
        with open(self.filename, 'wb') as f:  # 以二进制写入方式打开文件
            pickle.dump(self.poses, f)  # 序列化字典并写入文件
        print("Saved poses to disk.")  # 提示保存成功

    def load_poses(self):  # 从文件加载保存的位姿
        if os.path.exists(self.filename):  # 如果文件存在
            with open(self.filename, 'rb') as f:  # 以二进制读取方式打开文件
                self.poses = pickle.load(f)  # 反序列化加载字典
            print("Loaded poses from disk.")  # 提示加载成功
        else:
            print("No saved poses found on disk.")  # 提示没有找到文件

    def run(self):  # 主运行函数，提供命令行交互
        print("Welcome to the map annotator!")  # 打印欢迎信息
        print("Commands:\n"
              "  list: List saved poses.\n"
              "  save <name>: Save the robot's current pose as <name>.\n"
              "  delete <name>: Delete the pose given by <name>.\n"
              "  goto <name>: Send the robot to the pose given by <name>.\n"
              "  help: Show this list of commands.\n"
              "  exit: Save and exit.\n")  # 打印命令列表

        while not rospy.is_shutdown():  # 循环直到节点关闭
            try:
                command = input("> ").strip()  # 等待用户输入命令并去除首尾空格
                if command == "list":
                    self.list_poses()  # 执行 list 命令
                elif command.startswith("save "):
                    _, name = command.split(" ", 1)  # 解析名字
                    self.save_pose(name)  # 保存位姿
                elif command.startswith("delete "):
                    _, name = command.split(" ", 1)  # 解析名字
                    self.delete_pose(name)  # 删除位姿
                elif command.startswith("goto "):
                    _, name = command.split(" ", 1)  # 解析名字
                    self.goto_pose(name)  # 导航到位姿
                elif command == "help":
                    print("Commands:\n"
                          "  list: List saved poses.\n"
                          "  save <name>: Save the robot's current pose as <name>.\n"
                          "  delete <name>: Delete the pose given by <name>.\n"
                          "  goto <name>: Send the robot to the pose given by <name>.\n"
                          "  help: Show this list of commands.\n"
                          "  exit: Save and exit.\n")  # 打印命令帮助
                elif command == "exit":
                    self.save_to_disk()  # 保存到文件
                    break  # 退出循环
                else:
                    print("Unknown command. Type 'help' for list of commands.")  # 提示未知命令
            except (KeyboardInterrupt, EOFError):  # 捕捉 Ctrl+C 或 EOF (例如 Ctrl+D)
                self.save_to_disk()  # 保存到文件
                print("\nExiting.")  # 提示退出
                break  # 退出循环

if __name__ == '__main__':  # 如果作为主程序运行
    annotator = MapAnnotator()  # 创建 MapAnnotator 实例
    annotator.run()  # 调用运行函数



    
