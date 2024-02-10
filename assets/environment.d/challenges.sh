#!/usr/bin/env bash

copy-ros-logs() {
    find /tmp/log  -type f  -name "*.log" -exec cp {} /challenges/challenge-solution-output \;
}
