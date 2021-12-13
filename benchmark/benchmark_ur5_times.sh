#!/bin/bash

cd "${0%/*}"

dfolder=package://motion_bench_maker/problems/
rfolder=package://motion_bench_maker/benchmark/results/
planners="BKPIECEkConfigDefault, RRTConnectkConfigDefault, BiESTkConfigDefault"

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_small_ur5/" dataset:=$dfolder"bookshelf_small_ur5/" planners:="$planners" exp_name:="small_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_tall_ur5/" dataset:=$dfolder"bookshelf_tall_ur5/" planners:="$planners" exp_name:="tall_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_bars_ur5/" dataset:=$dfolder"table_bars_ur5/" planners:="$planners" exp_name:="bars_ur5" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"bookshelf_thin_ur5/" dataset:=$dfolder"bookshelf_thin_ur5/" planners:="$planners" exp_name:="thin_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"cage_ur5/" dataset:=$dfolder"cage_ur5/" planners:="$planners" exp_name:="cage_ur5" &

wait

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"box_ur5/" dataset:=$dfolder"box_ur5/" planners:="$planners"  exp_name:="box_ur5"  &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_pick_ur5/" dataset:=$dfolder"table_pick_ur5/" planners:="$planners" exp_name:="table_ur5" &
wait 

roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"table_under_pick_ur5/" dataset:=$dfolder"table_under_pick_ur5/" planners:="$planners" exp_name:="table_under_ur5" &
roslaunch --wait motion_bench_maker benchmark.launch end:=100 time:=60  runs:=20  sensed:=false results:=$rfolder"kitchen_ur5/" dataset:=$dfolder"kitchen_ur5/" planners:="$planners" exp_name:="kitchen_ur5" &

wait

killall roscore 

#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_tall_ur5/"    dataset:=$dfolder"bookshelf_tall_ur5/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"bookshelf_thin_ur5/"    dataset:=$dfolder"bookshelf_thin_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"box_ur5/"               dataset:=$dfolder"box_ur5/" &
#wait                                                                                                    
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"cage_ur5/"              dataset:=$dfolder"cage_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_pick_ur5/"        dataset:=$dfolder"table_pick_ur5/" &
#wait                                                                                                   
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"table_under_pick_ur5/"  dataset:=$dfolder"table_under_pick_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=false results:=$rfolder"kitchen_ur5/"           dataset:=$dfolder"kitchen_ur5/" &
#wait
#
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_small_ur5/"   dataset:=$dfolder"bookshelf_small_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_tall_ur5/"    dataset:=$dfolder"bookshelf_tall_ur5/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_bookshelf_thin_ur5/"    dataset:=$dfolder"bookshelf_thin_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_box_ur5/"               dataset:=$dfolder"box_ur5/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_cage_ur5/"              dataset:=$dfolder"cage_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_pick_ur5/"        dataset:=$dfolder"table_pick_ur5/" &
#wait                                                                                                
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_table_under_pick_ur5/"  dataset:=$dfolder"table_under_pick_ur5/" &
#roslaunch --wait motion_bench_maker benchmark.launch  sensed:=true results:=$rfolder"sensed_kitchen_ur5/"           dataset:=$dfolder"kitchen_ur5/" &
#wait
# kill everything
#ompl_benchmark_statistics.py results/bookshelf_small_ur5_results.log -d bookshelf_small.db
