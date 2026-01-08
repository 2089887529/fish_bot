#! /usr/bin/env python3
import time
from geometry_msgs.msg import PoseStamped 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration

"""
副驾驶注释：
这个脚本实现了让机器人从原点出发，移动到 X=1.5, Y=0.5 的位置。
"""

def main():
    # 1. 初始化 ROS 2 上下文
    rclpy.init()
    
    # 2. 实例化 Nav2 操作员 (这就是 API 的核心对象)
    navigator = BasicNavigator()

    # 3. 等待 Nav2 系统完全启动
    # 这一步会检查 amcl, map_server, planner, controller 是否都在活跃状态
    # 如果 Nav2 没启动好，脚本会卡在这里等待，非常安全
    print("等待 Nav2 启动中...")
    # navigator.waitUntilNav2Active() 

    # =====================================================
    # 阶段一：设置初始位置 (代替 Rviz 的 "2D Pose Estimate")
    # =====================================================
    # 注意：如果你的 AMCL 已经定位准了，这步其实可以跳过
    # 但为了自动化，我们通常会在脚本里强制重置一下位置
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.orientation.z = 0.0
    initial_pose.pose.orientation.w = 1.0
    
    print("正在初始化位置 (Set Initial Pose)...")
    navigator.setInitialPose(initial_pose)
    
    # 等待 2 秒让 AMCL 粒子散开并收敛
    time.sleep(2.0)

    # =====================================================
    # 阶段二：发送导航目标 (代替 Rviz 的 "2D Goal Pose")
    # =====================================================
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    
    # --- 【这里修改你想去的坐标】 ---
    # 你的地图是六边形，注意不要设到墙里去了
    goal_pose.pose.position.x = 1.5
    goal_pose.pose.position.y = 0.5
    goal_pose.pose.orientation.w = 1.0 # 朝向默认

    print(f"发布目标点: ({goal_pose.pose.position.x}, {goal_pose.pose.position.y})")
    navigator.goToPose(goal_pose)

    # =====================================================
    # 阶段三：监控导航过程 (反馈循环)
    # =====================================================
    while not navigator.isTaskComplete():
        # 获取反馈 (比如还剩多少米，导航了多久)
        feedback = navigator.getFeedback()
        
        # 打印剩余距离 (每秒刷屏太快，这里仅作示例，实际可加延时)
        # print(f'剩余距离: {feedback.distance_remaining:.2f} 米')
        
        # 这里可以加超时处理：如果超过 3 分钟没到，就取消
        # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=180.0):
        #     navigator.cancelTask()

    # =====================================================
    # 阶段四：处理最终结果
    # =====================================================
    result = navigator.getResult()
    
    if result == TaskResult.SUCCEEDED:
        print('✅ 任务完成！外卖已送达！')
    elif result == TaskResult.CANCELED:
        print('❌ 任务被取消！')
    elif result == TaskResult.FAILED:
        print('⚠️ 任务失败！可能是规划不出路径，或者被障碍物挡住了。')

    exit(0)

if __name__ == '__main__':
    main()