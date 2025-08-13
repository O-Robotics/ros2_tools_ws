#!/bin/bash

# ROS2 Bag录制脚本
# 使用方法: ./record_ros2_bag.sh [录制时长(秒)] [自定义名称]
# 示例: ./record_ros2_bag.sh 60 gnss_test

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# 默认参数
DURATION=${1:-60}  # 默认录制60秒
CUSTOM_NAME=${2:-"sensor_data"}  # 默认名称
OUTPUT_DIR="/home/alfol/ros2_bags"
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="${CUSTOM_NAME}_${TIMESTAMP}"

echo -e "${BLUE}=== ROS2 Bag录制脚本 ===${NC}"
echo -e "${YELLOW}录制时长: ${DURATION}秒${NC}"
echo -e "${YELLOW}输出目录: ${OUTPUT_DIR}${NC}"
echo -e "${YELLOW}Bag文件名: ${BAG_NAME}${NC}"

# 1. Source ROS2环境
echo -e "\n${BLUE}步骤1: Source ROS2环境${NC}"
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 Humble环境已加载${NC}"
else
    echo -e "${RED}✗ 未找到ROS2 Humble环境${NC}"
    exit 1
fi

# 2. Source工作空间
echo -e "\n${BLUE}步骤2: Source工作空间${NC}"
WORKSPACE_DIR="/home/alfol/Documents/ros2_localization_ws"
if [ -f "${WORKSPACE_DIR}/install/setup.bash" ]; then
    source ${WORKSPACE_DIR}/install/setup.bash
    echo -e "${GREEN}✓ 工作空间环境已加载${NC}"
else
    echo -e "${RED}✗ 工作空间未构建或setup.bash不存在${NC}"
    echo -e "${YELLOW}请先运行: cd ${WORKSPACE_DIR} && colcon build${NC}"
    exit 1
fi

# 3. 检查ROS2守护进程
echo -e "\n${BLUE}步骤3: 检查ROS2守护进程${NC}"
if ! pgrep -f "ros2 daemon" > /dev/null; then
    echo -e "${YELLOW}启动ROS2守护进程...${NC}"
    ros2 daemon start
    sleep 2
fi
echo -e "${GREEN}✓ ROS2守护进程运行中${NC}"

# 4. 显示可用话题
echo -e "\n${BLUE}步骤4: 显示当前可用话题${NC}"
echo -e "${YELLOW}当前活跃的话题:${NC}"
ros2 topic list

# 5. 创建输出目录
echo -e "\n${BLUE}步骤5: 准备输出目录${NC}"
mkdir -p "${OUTPUT_DIR}/${BAG_NAME}"
echo -e "${GREEN}✓ 输出目录已创建: ${OUTPUT_DIR}/${BAG_NAME}${NC}"

# 6. 开始录制
echo -e "\n${BLUE}步骤6: 开始录制ROS2 bag${NC}"
echo -e "${YELLOW}录制将在3秒后开始...${NC}"
sleep 1
echo -e "${YELLOW}3...${NC}"
sleep 1
echo -e "${YELLOW}2...${NC}"
sleep 1
echo -e "${YELLOW}1...${NC}"
sleep 1
echo -e "${GREEN}开始录制!${NC}"

# 定义要录制的话题 - 根据你的GNSS定位系统调整
TOPICS_TO_RECORD=(
    "/fix"                    # GPS fix信息
    "/imu/data_raw"           # 原始IMU数据
    # "/rtcm"                   # RTCM差分数据
    # "/diagnostics"            # 诊断信息
    # "/tf"                     # 坐标变换
    # "/tf_static"              # 静态坐标变换
)

# 构建录制命令
RECORD_CMD="ros2 bag record"
for topic in "${TOPICS_TO_RECORD[@]}"; do
    # 检查话题是否存在
    if ros2 topic list | grep -q "^${topic}$"; then
        RECORD_CMD="${RECORD_CMD} ${topic}"
        echo -e "${GREEN}✓ 将录制话题: ${topic}${NC}"
    else
        echo -e "${YELLOW}⚠ 话题不存在，跳过: ${topic}${NC}"
    fi
done

# 添加输出目录和其他参数
RECORD_CMD="${RECORD_CMD} -o ${OUTPUT_DIR}/${BAG_NAME}"

# 如果指定了录制时长，添加duration参数
if [ "$DURATION" != "0" ]; then
    RECORD_CMD="${RECORD_CMD} --max-bag-duration ${DURATION}"
fi

echo -e "\n${BLUE}执行命令:${NC}"
echo -e "${YELLOW}${RECORD_CMD}${NC}"

# 执行录制
eval $RECORD_CMD &
RECORD_PID=$!

# 显示录制状态
if [ "$DURATION" != "0" ]; then
    echo -e "\n${GREEN}正在录制... (${DURATION}秒)${NC}"
    echo -e "${YELLOW}按Ctrl+C可提前停止录制${NC}"
    
    # 等待录制完成或用户中断
    for ((i=1; i<=DURATION; i++)); do
        if ! kill -0 $RECORD_PID 2>/dev/null; then
            break
        fi
        echo -ne "\r${BLUE}录制进度: ${i}/${DURATION}秒${NC}"
        sleep 1
    done
    echo ""
else
    echo -e "\n${GREEN}正在录制... (无时长限制)${NC}"
    echo -e "${YELLOW}按Ctrl+C停止录制${NC}"
    wait $RECORD_PID
fi

# 等待进程结束
sleep 2

# 7. 验证录制结果
echo -e "\n${BLUE}步骤7: 验证录制结果${NC}"
if [ -d "${OUTPUT_DIR}/${BAG_NAME}" ]; then
    echo -e "${GREEN}✓ Bag文件已保存到: ${OUTPUT_DIR}/${BAG_NAME}${NC}"
    
    # 显示文件信息
    echo -e "\n${BLUE}文件信息:${NC}"
    ls -la "${OUTPUT_DIR}/${BAG_NAME}/"
    
    # 显示bag信息
    echo -e "\n${BLUE}Bag信息:${NC}"
    ros2 bag info "${OUTPUT_DIR}/${BAG_NAME}"
    
else
    echo -e "${RED}✗ 录制失败，未找到输出文件${NC}"
    exit 1
fi

echo -e "\n${GREEN}=== 录制完成! ===${NC}"
echo -e "${BLUE}Bag文件位置: ${OUTPUT_DIR}/${BAG_NAME}${NC}"
echo -e "${BLUE}使用以下命令播放:${NC}"
echo -e "${YELLOW}ros2 bag play ${OUTPUT_DIR}/${BAG_NAME}${NC}"
