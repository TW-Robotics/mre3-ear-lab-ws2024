#!/bin/bash

if [ "$(pgrep roscore)" ] || [ "$(pgrep rosmaster)" ]; then
    datum="$(date +%d_%m_%Y)"
    path=$HOME/maps/
    mkdir -p "$path""$datum"
    cd "$path""$datum" || exit 1
    cnt="$(find "$(cd ..; pwd)" -type f -name "*.pbstream" | wc -l)"
    echo -e "\e[32mSaving current cartographer map to  $path$datum\e[0m"

    rosservice call /finish_trajectory "trajectory_id: 0"
    rosservice call /write_state "filename: $path$datum/Map_$cnt.pbstream
include_unfinished_submaps: false"

    echo -e "\e[32mFinished\e[0m"
else
    echo -e "\e[31mNo roscore running -> aborting\e[0m"
fi
