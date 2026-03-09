#!/bin/bash
# 如果任何命令出錯，立即退出 (保護機制)
set -eo pipefail

ROS_DISTRO=${ROS_DISTRO:-noetic}
WORKSPACE=/root
BASHRC_FILE="${WORKSPACE}/.bashrc"
# [新增] 預設開啟 roscore，也可以透過 docker-compose environment 設為 false 關閉
AUTO_ROSCORE=${AUTO_ROSCORE:-true}

echo "========================================="
echo "   ROS ${ROS_DISTRO} Hybrid Container"
echo "========================================="

# -------------------------------------------------
# Helper 函數：檢查並決定是否編譯
# -------------------------------------------------
build_workspace() {
    local ws_path=$1
    local build_type=$2 # "catkin_make" or "catkin_make_isolated"

    if [ -d "${ws_path}/src" ]; then
        echo "=== Checking $(basename $ws_path) ==="
        
        local setup_file="${ws_path}/devel/setup.bash"
        if [ "$build_type" == "catkin_make_isolated" ]; then
            setup_file="${ws_path}/devel_isolated/setup.bash"
        fi

        if [ ! -f "$setup_file" ]; then
            echo ">>> $(basename $ws_path) not built (Dev Mode detected). Building now..."
            
            if [ "$build_type" == "catkin_make" ] && [ -f "${ws_path}/src/CMakeLists.txt" ]; then
                echo "   -> Removing potential broken CMakeLists.txt symlink..."
                rm "${ws_path}/src/CMakeLists.txt"
            fi

            cd ${ws_path}
            $build_type -j$(nproc)
        else
            echo ">>> $(basename $ws_path) already built. Skipping compilation."
        fi

        if [ -f "$setup_file" ]; then
            # 1. 載入到當前腳本環境 (這對接下來啟動 roscore 很重要)
            source $setup_file
            
            # 2. 寫入 .bashrc
            local source_cmd="source $setup_file --extend"
            if ! grep -Fxq "$source_cmd" $BASHRC_FILE; then
                echo "$source_cmd" >> $BASHRC_FILE
                echo "   -> Added to .bashrc with --extend"
            fi
        fi
    else
        echo "!!! WARNING: $(basename $ws_path)/src not found. Skipping."
    fi
}

# -------------------------------------------------
# 1. Source System ROS & Setup .bashrc
# -------------------------------------------------
source /opt/ros/${ROS_DISTRO}/setup.bash

if ! grep -Fxq "source /opt/ros/${ROS_DISTRO}/setup.bash" $BASHRC_FILE; then
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> $BASHRC_FILE
fi

# -------------------------------------------------
# 2. 依序處理所有工作空間
# -------------------------------------------------
build_workspace "${WORKSPACE}/hdl_ws" "catkin_make"
build_workspace "${WORKSPACE}/lidar_ws" "catkin_make_isolated"
build_workspace "${WORKSPACE}/realsense_ws" "catkin_make"
build_workspace "${WORKSPACE}/keyboard_control_ws" "catkin_make"

# -------------------------------------------------
# 2.5 [新增] 自動啟動 Roscore
# -------------------------------------------------
if [ "$AUTO_ROSCORE" = "true" ]; then
    echo "-----------------------------------------"
    echo "   Starting Local Roscore"
    echo "-----------------------------------------"
    
    # 在背景啟動 roscore
    roscore &
    ROSCORE_PID=$!
    
    # 等待 roscore 初始化 (最多等待 10 秒)
    echo "Waiting for roscore to initialize..."
    TIMEOUT=10
    COUNT=0
    until rostopic list > /dev/null 2>&1; do
        if [ "$COUNT" -ge "$TIMEOUT" ]; then
            echo "!!! Timeout waiting for roscore !!!"
            # 即使超時也不退出，讓使用者可以進入容器 debug
            break 
        fi
        sleep 1
        COUNT=$((COUNT+1))
        echo -n "."
    done
    echo ""
    echo ">>> Roscore started (PID: $ROSCORE_PID)"
fi

# -------------------------------------------------
# 3. 執行指令
# -------------------------------------------------
source $BASHRC_FILE
echo "=== Environment ready ==="

if [ $# -gt 0 ]; then
    # 使用 exec 替換進程，確保訊號傳遞正常
    exec "$@"
else
    # 如果沒有指令，預設進入 bash
    exec bash
fi