#!/bin/bash
declare -a arr=({{.Array}})
cd ~/catkin_ws/src
for i in "${arr[@]}"
do
    foldername=$(basename $i)
    if ! [ -d $foldername"/.git" ]; then
        git clone $i
        cd $foldername
    else
        cd $foldername
        git pull
    fi
    if test -f "canopy.yml"; then
        cd ~/catkin_ws
        python /canopy_remote_run.py src/$foldername/canopy.yml
        cd ~/catkin_ws/src
    fi
done
