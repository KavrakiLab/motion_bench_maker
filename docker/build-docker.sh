#!/bin/bash

cd "${0%/*}"
docker build -t motion_bench_maker . `git rev-parse --show-toplevel`
