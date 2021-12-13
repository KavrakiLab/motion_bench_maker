#!/bin/bash
dfolder=package://motion_bench_maker/problems/
cfolder=package://motion_bench_maker/configs/problems/

roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"box_panda.yaml" dataset:=$dfolder"box_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_small_panda.yaml" dataset:=$dfolder"bookshelf_small_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_tall_panda.yaml" dataset:=$dfolder"bookshelf_tall_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_thin_panda.yaml" dataset:=$dfolder"bookshelf_thin_panda/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"cage_panda.yaml" dataset:=$dfolder"cage_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_pick_panda.yaml" dataset:=$dfolder"table_pick_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_under_pick_panda.yaml" dataset:=$dfolder"table_under_pick_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"kitchen_panda.yaml" dataset:=$dfolder"kitchen_panda/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_bars_panda.yaml" dataset:=$dfolder"table_bars_panda/" &
wait
