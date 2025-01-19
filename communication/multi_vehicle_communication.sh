#!/bin/bash

# 这个脚本的主要功能是：
# 首先定义了8种不同类型的无人机/车辆的数量
# 然后通过多个while循环，分别启动对应数量的各类型载具的通信程序
# 每个通信程序都在后台运行（使用&符号）
# 使用vehicle_num作为索引来区分同类型的不同载具
# 所有的Python脚本都是并行执行的，这样可以同时控制多个载具
# 这个脚本看起来是用于多机器人系统的仿真环境中，可以同时启动多个不同类型的无人机或车辆的通信程序。

# 设置不同类型无人机的数量
iris_num=3                # Iris四旋翼无人机数量
typhoon_h480_num=0       # Typhoon H480六旋翼无人机数量
solo_num=0               # Solo四旋翼无人机数量
plane_num=0              # 固定翼飞机数量
rover_num=0              # 地面车辆数量
standard_vtol_num=0      # 标准垂直起降机数量
tiltrotor_num=0          # 倾转旋翼机数量
tailsitter_num=0         # 尾座式垂直起降机数量

# 启动Iris四旋翼无人机仿真
vehicle_num=0
while(( $vehicle_num< iris_num)) 
do
    python multirotor_communication.py iris $vehicle_num&    # 后台运行Iris无人机通信程序
    let "vehicle_num++"
done

# 启动Typhoon H480无人机仿真
vehicle_num=0
while(( $vehicle_num< typhoon_h480_num)) 
do
    python multirotor_communication.py typhoon_h480 $vehicle_num&    # 后台运行Typhoon无人机通信程序
    let "vehicle_num++"
done

# 启动Solo四旋翼无人机
vehicle_num=0
while(( $vehicle_num< solo_num)) 
do
    python multirotor_communication.py solo $vehicle_num&
    let "vehicle_num++"
done

# 启动固定翼飞机仿真
vehicle_num=0
while(( $vehicle_num< plane_num)) 
do
    python plane_communication.py $vehicle_num&    # 后台运行固定翼飞机通信程序
    let "vehicle_num++"
done

# 启动地面车辆仿真
vehicle_num=0
while(( $vehicle_num< rover_num)) 
do
    python rover_communication.py $vehicle_num&
    let "vehicle_num++"
done

# 启动垂直起降机仿真
vehicle_num=0
while(( $vehicle_num< standard_vtol_num)) 
do
    python vtol_communication.py standard_vtol $vehicle_num&    # 后台运行标准VTOL通信程序
    let "vehicle_num++"
done

# 启动倾转旋翼机仿真
vehicle_num=0
while(( $vehicle_num< tiltrotor_num)) 
do
    python vtol_communication.py tiltrotor $vehicle_num&
    let "vehicle_num++"
done

# 启动尾座式垂直起降机仿真
vehicle_num=0
while(( $vehicle_num< tailsitter_num)) 
do
    python vtol_communication.py tailsitter $vehicle_num&
    let "vehicle_num++"
done
