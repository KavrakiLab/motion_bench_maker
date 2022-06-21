#!/bin/bash

cd "${0%/*}"
docker build -t motion_bench_maker -f Dockerfile `git rev-parse --show-toplevel`
