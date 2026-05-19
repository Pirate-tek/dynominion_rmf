#!/bin/bash

ST2=configure
ST3=activate
for i in {1..5}; do
NODE="/dynominion$i/amcl"
STATE=$(ros2 lifecycle get $NODE | awk '{print $1}')
echo $NODE $STATE
if [ "$STATE" = "unconfigured" ]; then
ros2 lifecycle set $NODE  $ST2
ros2 lifecycle set $NODE $ST3
fi
done
