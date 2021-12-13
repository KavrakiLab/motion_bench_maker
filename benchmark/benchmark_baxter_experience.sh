#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/
planners="ThunderConfig"

roslaunch --wait motion_bench_maker benchmark.launch end:=50 train:=true time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_easy_baxter/" planners:="$planners" exp_name:="tall_baxter_easy_50" &
roslaunch --wait motion_bench_maker benchmark.launch end:=50 train:=true time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_medium_baxter/" planners:="$planners" exp_name:="tall_baxter_medium_50" &
roslaunch --wait motion_bench_maker benchmark.launch end:=50 train:=true time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_hard_baxter/" planners:="$planners" exp_name:="tall_baxter_hard_50" &

wait

roslaunch --wait motion_bench_maker benchmark.launch start:=50 end:=100 train:=false time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_easy_baxter/" planners:="$planners" exp_name:="tall_baxter_easy_50" &
roslaunch --wait motion_bench_maker benchmark.launch start:=50 end:=100 train:=false time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_medium_baxter/" planners:="$planners" exp_name:="tall_baxter_medium_50" &
roslaunch --wait motion_bench_maker benchmark.launch end:=50 train:=false time:=60  runs:=2 sensed:=false results:=$rfolder"tall_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_hard_baxter/" planners:="$planners" exp_name:="tall_baxter_hard_50" &

killall roscore 
