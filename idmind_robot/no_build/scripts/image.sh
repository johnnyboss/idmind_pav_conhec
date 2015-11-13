#!/bin/bash

img=$1
folder="/catkin_ws/src/pavilhao_conhecimento/media/images/"
img_path=$HOME$folder$img

eog --fullscreen $img_path &

exit 0
