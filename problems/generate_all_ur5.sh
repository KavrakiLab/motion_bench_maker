#!/bin/bash
dfolder=package://motion_bench_maker/problems/
cfolder=package://motion_bench_maker/configs/problems/

roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_small_ur5.yaml" dataset:=$dfolder"bookshelf_small_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_tall_ur5.yaml" dataset:=$dfolder"bookshelf_tall_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_thin_ur5.yaml" dataset:=$dfolder"bookshelf_thin_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"box_ur5.yaml" dataset:=$dfolder"box_ur5/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"cage_ur5.yaml" dataset:=$dfolder"cage_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_pick_ur5.yaml" dataset:=$dfolder"table_pick_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_under_pick_ur5.yaml" dataset:=$dfolder"table_under_pick_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"kitchen_ur5.yaml" dataset:=$dfolder"kitchen_ur5/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_bars_ur5.yaml" dataset:=$dfolder"table_bars_ur5/" &
wait
