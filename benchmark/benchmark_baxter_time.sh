#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/
planners="BKPIECEkConfigDefault, RRTConnectkConfigDefault, BiESTkConfigDefault"

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_small_baxter/" dataset:=$dfolder"bookshelf_small_baxter/" planners:="$planners" exp_name:="small_baxter" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_tall_baxter/" dataset:=$dfolder"bookshelf_tall_baxter/" planners:="$planners" exp_name:="tall_baxter" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_bars_baxter/" dataset:=$dfolder"table_bars_baxter/" planners:="$planners" exp_name:="bars_baxter" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_thin_baxter/" dataset:=$dfolder"bookshelf_thin_baxter/" planners:="$planners" exp_name:="thin_baxter" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"cage_baxter/" dataset:=$dfolder"cage_baxter/" planners:="$planners" exp_name:="cage_baxter" &

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_tall_both_arms_easy_baxter/" dataset:=$dfolder"bookshelf_tall_both_arms_easy_baxter/" planners:="$planners" exp_name:="bookshelf_tall_both_arms_easy_baxter/" &


wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"box_baxter/" dataset:=$dfolder"box_baxter/" planners:="$planners"  exp_name:="box_baxter"  &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_pick_baxter/" dataset:=$dfolder"table_pick_baxter/" planners:="$planners" exp_name:="table_baxter" &
wait 

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_under_pick_baxter/" dataset:=$dfolder"table_under_pick_baxter/" planners:="$planners" exp_name:="table_under_baxter" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"kitchen_baxter/" dataset:=$dfolder"kitchen_baxter/" planners:="$planners" exp_name:="kitchen_baxter" &

wait

killall roscore 

#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_tall_baxter/"    dataset:=$dfolder"bookshelf_tall_baxter/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_thin_baxter/"    dataset:=$dfolder"bookshelf_thin_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"box_baxter/"               dataset:=$dfolder"box_baxter/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"cage_baxter/"              dataset:=$dfolder"cage_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_pick_baxter/"        dataset:=$dfolder"table_pick_baxter/" &
#wait                                                                                                   
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_under_pick_baxter/"  dataset:=$dfolder"table_under_pick_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"kitchen_baxter/"           dataset:=$dfolder"kitchen_baxter/" &
#wait
#
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_small_baxter/"   dataset:=$dfolder"bookshelf_small_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_tall_baxter/"    dataset:=$dfolder"bookshelf_tall_baxter/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_thin_baxter/"    dataset:=$dfolder"bookshelf_thin_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_box_baxter/"               dataset:=$dfolder"box_baxter/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_cage_baxter/"              dataset:=$dfolder"cage_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_pick_baxter/"        dataset:=$dfolder"table_pick_baxter/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_under_pick_baxter/"  dataset:=$dfolder"table_under_pick_baxter/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_kitchen_baxter/"           dataset:=$dfolder"kitchen_baxter/" &
#wait
# kill everything
#ompl_benchmark_statistics.py results/bookshelf_small_baxter_results.log -d bookshelf_small.db
