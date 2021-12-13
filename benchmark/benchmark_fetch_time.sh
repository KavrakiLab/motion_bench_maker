#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/
planners="RRTConnect_DEFAULT, BKPIECE_DEFAULT, BiEST_DEFAULT"

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_small_fetch/" dataset:=$dfolder"bookshelf_small_fetch/" planners:="$planners" exp_name:="small_fetch" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_tall_fetch/" dataset:=$dfolder"bookshelf_tall_fetch/" planners:="$planners" exp_name:="tall_fetch" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_bars_fetch/" dataset:=$dfolder"table_bars_fetch/" planners:="$planners" exp_name:="bars_fetch" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_thin_fetch/" dataset:=$dfolder"bookshelf_thin_fetch/" planners:="$planners" exp_name:="thin_fetch" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"cage_fetch/" dataset:=$dfolder"cage_fetch/" planners:="$planners" exp_name:="cage_fetch" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"box_fetch/" dataset:=$dfolder"box_fetch/" planners:="$planners"  exp_name:="box_fetch"  &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_pick_fetch/" dataset:=$dfolder"table_pick_fetch/" planners:="$planners" exp_name:="table_fetch" &
wait 

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_under_pick_fetch/" dataset:=$dfolder"table_under_pick_fetch/" planners:="$planners" exp_name:="table_under_fetch" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"kitchen_fetch/" dataset:=$dfolder"kitchen_fetch/" planners:="$planners" exp_name:="kitchen_fetch" &

wait

killall roscore 

#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_tall_fetch/"    dataset:=$dfolder"bookshelf_tall_fetch/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_thin_fetch/"    dataset:=$dfolder"bookshelf_thin_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"box_fetch/"               dataset:=$dfolder"box_fetch/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"cage_fetch/"              dataset:=$dfolder"cage_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_pick_fetch/"        dataset:=$dfolder"table_pick_fetch/" &
#wait                                                                                                   
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_under_pick_fetch/"  dataset:=$dfolder"table_under_pick_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"kitchen_fetch/"           dataset:=$dfolder"kitchen_fetch/" &
#wait
#
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_small_fetch/"   dataset:=$dfolder"bookshelf_small_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_tall_fetch/"    dataset:=$dfolder"bookshelf_tall_fetch/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_thin_fetch/"    dataset:=$dfolder"bookshelf_thin_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_box_fetch/"               dataset:=$dfolder"box_fetch/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_cage_fetch/"              dataset:=$dfolder"cage_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_pick_fetch/"        dataset:=$dfolder"table_pick_fetch/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_under_pick_fetch/"  dataset:=$dfolder"table_under_pick_fetch/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_kitchen_fetch/"           dataset:=$dfolder"kitchen_fetch/" &
#wait
# kill everything
#ompl_benchmark_statistics.py results/bookshelf_small_fetch_results.log -d bookshelf_small.db
