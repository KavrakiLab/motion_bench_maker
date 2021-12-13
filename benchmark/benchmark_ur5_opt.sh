#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/

planner1="BITstarkConfigDefault,"
planner2="AITstarkConfigDefault,"
planner3="RRTstarkConfigDefault"
planner4="PRMstarkConfigDefault"

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=5 sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:=$planner1  exp_name:="box_ur5_A"  &

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=5 sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:=$planner2 exp_name:="box_ur5_B"  &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=5 sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:=$planner3 exp_name:="box_ur5_C"  &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=5 sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:=$planner4 exp_name:="box_ur5_D"  &


#roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=20  runs:=10  sensed:=false results:=$rfolder"bookshelf_small_ur5/" dataset:=$dfolder"bookshelf_small_ur5/" planners:="$planners" exp_name:="small_ur5" &
#roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=20  runs:=10  sensed:=false results:=$rfolder"table_bars_ur5/" dataset:=$dfolder"table_bars_ur5/" planners:="$planners" exp_name:="bars_ur5" &
#roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=20  runs:=10  sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:="$planners"  exp_name:="box_ur5"  &
#roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=20  runs:=10  sensed:=false results:=$rfolder"table_under_pick_ur5/" dataset:=$dfolder"table_under_pick_ur5/" planners:="$planners" exp_name:="table_under_ur5" &
#roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=20  runs:=10  sensed:=false results:=$rfolder"kitchen_ur5/" dataset:=$dfolder"kitchen_ur5/" planners:="$planners" exp_name:="kitchen_ur5" &


wait 

killall roscore 
