#!/bin/bash

# 檢查是否提供了參數
if [ $# -eq 0 ]; then
    echo "Usage: $0 <launcher_number>"
    exit 1
fi

# 獲取參數
LAUNCHER_NUMBER=$1

# 設置 ROS 環境
source /opt/ros/noetic/setup.bash
source ./devel/setup.bash
#TARGET_DIR=~/groundtruth_compare/data

# 確認目錄是否存在
#if [ ! -d "$TARGET_DIR" ]; then
 #   echo "Directory $TARGET_DIR does not exist."
  #  exit 1
#fi

# 刪除目錄中的所有文件
#rm -rf "$TARGET_DIR"/*

# 確認操作完成
#echo "All files in $TARGET_DIR have been deleted."


# 啟動指定的 launcher
roslaunch floam floam_mapping_${LAUNCHER_NUMBER}.launch
