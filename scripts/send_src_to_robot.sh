#!/bin/bash
SRC_PATH=$(realpath $(dirname "$(realpath $0)")/../)/
echo Transferring path $SRC_PATH
rsync -av --delete --exclude .git/ $SRC_PATH user@192.168.1.102:/home/user/roskit_ws/src
# optionally --delete flag to remove all files that are not contained in the source folder
