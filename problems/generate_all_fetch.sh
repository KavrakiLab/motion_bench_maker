#!/bin/bash
dfolder=package://motion_bench_maker/problems/
cfolder=package://motion_bench_maker/configs/problems/

roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_small_fetch.yaml" dataset:=$dfolder"bookshelf_small_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_tall_fetch.yaml" dataset:=$dfolder"bookshelf_tall_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_thin_fetch.yaml" dataset:=$dfolder"bookshelf_thin_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"box_fetch.yaml" dataset:=$dfolder"box_fetch/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"cage_fetch.yaml" dataset:=$dfolder"cage_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_pick_fetch.yaml" dataset:=$dfolder"table_pick_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_under_pick_fetch.yaml" dataset:=$dfolder"table_under_pick_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"kitchen_fetch.yaml" dataset:=$dfolder"kitchen_fetch/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_bars_fetch.yaml" dataset:=$dfolder"table_bars_fetch/" &
wait
