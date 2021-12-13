#!/bin/bash
dfolder=package://motion_bench_maker/problems/
cfolder=package://motion_bench_maker/configs/problems/

roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_small_baxter.yaml" dataset:=$dfolder"bookshelf_small_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_tall_baxter.yaml" dataset:=$dfolder"bookshelf_tall_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"bookshelf_thin_baxter.yaml" dataset:=$dfolder"bookshelf_thin_baxter/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=false config:=$cfolder"bookshelf_tall_both_arms_easy_baxter.yaml" dataset:=$dfolder"bookshelf_tall_both_arms_easy_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=false config:=$cfolder"bookshelf_tall_both_arms_medium_baxter.yaml" dataset:=$dfolder"bookshelf_tall_both_arms_medium_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=false config:=$cfolder"bookshelf_tall_both_arms_baxter.yaml" dataset:=$dfolder"bookshelf_tall_both_arms_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_bars_baxter.yaml" dataset:=$dfolder"table_bars_baxter/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"kitchen_baxter.yaml" dataset:=$dfolder"kitchen_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_pick_baxter.yaml" dataset:=$dfolder"table_pick_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"box_baxter.yaml" dataset:=$dfolder"box_baxter/" &
wait
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"cage_baxter.yaml" dataset:=$dfolder"cage_baxter/" &
roslaunch --wait motion_bench_maker generate.launch  sensed:=true config:=$cfolder"table_under_pick_baxter.yaml" dataset:=$dfolder"table_under_pick_baxter/" &
wait
