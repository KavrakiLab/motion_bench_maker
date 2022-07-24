#!/bin/bash

cd "${0%/*}"
docker build -f Dockerfile -t motion_bench_maker_moveit_src ../
