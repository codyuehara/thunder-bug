#!/bin/bash

IMAGE_NAME=codyuehara/thunder-bug

help()
{
    echo ""
    echo "Usage: $0 -w dev_ws -t localhost/ros2-humble-dev -b"
    echo -e "\t-h show help"
    exit 1
}

while getopts "h" opt
do
    case "$opt" in
        h | ?) help ;;
    esac
done

xhost +local:docker
       #--gpus=all \
       #-v $PWD/.session.yml:/root/.session.yml \
       #-v $PWD/.tmux.conf:/root/.tmux.conf \
       #-v /home/buggy/charger-bug/vesc:/root/src/vesc \
docker run -it \
       --rm \
       --net=host \
       --privileged \
       -e DISPLAY=$DISPLAY \
       -e PYTHONBUFFERED=1 \
       -v /etc/timezone:/etc/timezone:ro \
       -v /etc/localtime:/etc/localtime:ro \
       -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
       -v $HOME/.Xauthority:/root/.Xauthority:ro \
       --device=/dev/bus/usb:/dev/bus/usb \
       $IMAGE_NAME
