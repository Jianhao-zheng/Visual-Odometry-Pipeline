#!/bin/bash

# File      :  gen_rosbag_yaml.sh
# Time      :  2021/12/07
# Author    :  Yujie He
# Contact   :  he-yujie@outlook.com
# State     :  Tested
# Reference :
# https://superuser.com/a/31466
# https://stackoverflow.com/a/10943265
# https://stackoverflow.com/a/52869469

# `xargs` removes the whitespace from the wc command
total=$(ls -1 ./*.bag | wc -l | xargs)

echo "Number of rosbag in currnet control type:" $total

n=0
for x in *.bag; do
    n=$((n + 1))
    t=${x%.bag}.yaml
    echo "($n/$total) generate $t from $x"
    rosbag info --yaml $x >$t
done
