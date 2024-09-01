#!/bin/bash

docker build --no-cache --rm $@ -t limo_bot:sim -f "$(dirname "$0")/../../docker/sim.Dockerfile" "$(dirname "$0")/../.."