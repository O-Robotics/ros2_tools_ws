#!/bin/bash

# 测试ROS2守护进程对性能的影响

echo "=== ROS2守护进程性能测试 ==="

# 停止守护进程
echo "1. 停止守护进程并测试..."
ros2 daemon stop
sleep 2

echo "没有守护进程时的话题列表查询时间:"
time ros2 topic list > /dev/null

# 启动守护进程
echo -e "\n2. 启动守护进程并测试..."
ros2 daemon start
sleep 3  # 等待守护进程完全启动

echo "有守护进程时的话题列表查询时间:"
time ros2 topic list > /dev/null

echo -e "\n3. 再次测试（守护进程已缓存）:"
time ros2 topic list > /dev/null

echo -e "\n=== 测试完成 ==="
echo "可以看到守护进程显著提升了命令执行速度"
