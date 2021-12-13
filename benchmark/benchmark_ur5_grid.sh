#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/
planners="RRTConnect_0_05, RRTConnect_0_25, RRTConnect_0_50, RRTConnect_0_75, RRTConnect_1_00, RRTConnect_1_25, RRTConnect_1_50, RRTConnect_1_75, RRTConnect_2_00, RRTConnect_2_25, RRTConnect_2_50, RRTConnect_2_75, RRTConnect_3_00, RRTConnect_3_25, RRTConnect_3_50, RRTConnect_3_75, RRTConnect_4_00, RRTConnect_4_25, RRTConnect_4_50, RRTConnect_4_75, RRTConnect_5_00, RRTConnect_5_25, RRTConnect_5_50, RRTConnect_5_75, RRTConnect_6_00, RRTConnect_6_25, RRTConnect_6_50, RRTConnect_6_75, RRTConnect_7_00, BiEST_0_05, BiEST_0_25, BiEST_0_50, BiEST_0_75, BiEST_1_00, BiEST_1_25, BiEST_1_50, BiEST_1_75, BiEST_2_00, BiEST_2_25, BiEST_2_50, BiEST_2_75, BiEST_3_00, BiEST_3_25, BiEST_3_50, BiEST_3_75, BiEST_4_00, BiEST_4_25, BiEST_4_50, BiEST_4_75, BiEST_5_00, BiEST_5_25, BiEST_5_50, BiEST_5_75, BiEST_6_00, BiEST_6_25, BiEST_6_50, BiEST_6_75, BiEST_7_00, BKPIECE_0_05, BKPIECE_0_25, BKPIECE_0_50, BKPIECE_0_75, BKPIECE_1_00, BKPIECE_1_25, BKPIECE_1_50, BKPIECE_1_75, BKPIECE_2_00, BKPIECE_2_25, BKPIECE_2_50, BKPIECE_2_75, BKPIECE_3_00, BKPIECE_3_25, BKPIECE_3_50, BKPIECE_3_75, BKPIECE_4_00, BKPIECE_4_25, BKPIECE_4_50, BKPIECE_4_75, BKPIECE_5_00, BKPIECE_5_25, BKPIECE_5_50, BKPIECE_5_75, BKPIECE_6_00, BKPIECE_6_25, BKPIECE_6_50, BKPIECE_6_75, BKPIECE_7_00"

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:="$planners"  exp_name:="box_ur5"  &

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"table_bars_ur5/" dataset:=$dfolder"table_bars_ur5/" planners:="$planners" exp_name:="bars_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"table_under_pick_ur5/" dataset:=$dfolder"table_under_pick_ur5/" planners:="$planners" exp_name:="table_under_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"kitchen_ur5/" dataset:=$dfolder"kitchen_ur5/" planners:="$planners" exp_name:="kitchen_ur5" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"bookshelf_small_ur5/" dataset:=$dfolder"bookshelf_small_ur5/" planners:="$planners" exp_name:="small_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"bookshelf_tall_ur5/" dataset:=$dfolder"bookshelf_tall_ur5/" planners:="$planners" exp_name:="tall_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"bookshelf_thin_ur5/" dataset:=$dfolder"bookshelf_thin_ur5/" planners:="$planners" exp_name:="thin_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=1  sensed:=false results:=$rfolder"cage_ur5/" dataset:=$dfolder"cage_ur5/" planners:="$planners" exp_name:="cage_ur5" &


wait

killall roscore
