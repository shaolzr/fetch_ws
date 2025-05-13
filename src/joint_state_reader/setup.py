## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from distutils.core import setup # 从 Python 的 distutils 库导入 setup，用于定义打包安装配置
from catkin_pkg.python_setup import generate_distutils_setup # 从 catkin_pkg 库导入 generate_distutils_setup
# 这个函数专门用来为 catkin（ROS 的构建系统）生成 distutils 配置

setup_args = generate_distutils_setup(
    packages=['joint_state_reader'],
    # 指定 Python 包名，这里是 joint_state_reader
    package_dir={'': 'src'}
    # 告诉 setup()：包代码位于 src 文件夹下
    # 也就是说 joint_state_reader 实际存放在 src/joint_state_reader 目录下
)

setup(**setup_args)
# 调用 setup() 并传入刚刚生成的配置参数，完成 setup 配置注册

"""
告诉 catkin 系统
→ 这个包是一个 Python 包，名字叫 joint_state_reader
→ 代码文件位置在 src/joint_state_reader
→ 当你用 catkin build 或 catkin_make 构建工作空间时，会自动识别和安装这个 Python 包
"""