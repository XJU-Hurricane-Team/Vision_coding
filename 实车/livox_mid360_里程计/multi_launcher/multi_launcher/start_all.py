import os
import subprocess

def main():
    print("🚀 正在为你唤醒并行终端...")


    # 终端 1：启动 Nav2
    cmd1 = f"gnome-terminal --title='Navigation2' -- bash -c 'source install/setup.bash && ros2 launch fishbot_navigation2 navigation2.launch.py; exec bash'"
    
    # 终端 2：启动 里程计坐标转换
    cmd2 = f"gnome-terminal --title='Odometry Transform' -- bash -c 'source install/setup.bash && ros2 launch odometry odometry_transform_launch.py; exec bash'"
    
    # 终端 3：启动 Python 脚本
    cmd3 = f"gnome-terminal --title='Odometry Transform' -- bash -c 'source install/setup.bash && ros2 run nav_speed_heading tracker_node; exec bash'"

    # 并行弹出三个终端
    subprocess.Popen(cmd1, shell=True)
    subprocess.Popen(cmd2, shell=True)
    subprocess.Popen(cmd3, shell=True)

    print("✅ 三个终端已成功弹出！你可以在弹出的窗口中查看各自的日志。")

if __name__ == '__main__':
    main()