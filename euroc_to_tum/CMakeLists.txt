cmake_minimum_required(VERSION 2.6)
project(euroc_to_tum)

add_executable(euroc_to_tum main.cpp)

install(TARGETS euroc_to_tum RUNTIME DESTINATION bin)

# vin-mono 输出的轨迹是 timestamps x y z  qx qy qz qw 
# tum建议的轨迹格式是 timestamps x y z qw qx qy qz 
# 使用evo工具分析轨迹的时候要用tum的格式 稍微转化一下就行
