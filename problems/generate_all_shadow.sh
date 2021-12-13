#!/bin/bash
dfolder=package://motion_bench_maker/problems/
cfolder=package://motion_bench_maker/configs/problems/

roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"bookshelf_small_shadowhand.yaml" dataset:=$dfolder"bookshelf_small_shadowhand/" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"bookshelf_tall_shadowhand.yaml" dataset:=$dfolder"bookshelf_tall_shadowhand/" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"bookshelf_thin_shadowhand.yaml" dataset:=$dfolder"bookshelf_thin_shadowhand/" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"box_shadowhand.yaml" dataset:=$dfolder"box_shadowhand/" &
wait

roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"cage_shadowhand.yaml" dataset:=$dfolder"cage_shadowhand" planner_name:="BKPIECE" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"table_under_pick_shadowhand.yaml" dataset:=$dfolder"table_under_pick_shadowhand/" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"kitchen_shadowhand.yaml" dataset:=$dfolder"kitchen_shadowhand/" &
roslaunch --wait motion_bench_maker generate.launch  end:=100 sensed:=true config:=$cfolder"table_bars_shadowhand.yaml" dataset:=$dfolder"table_bars_shadowhand/" &









