#!/bin/bash

sound=$1
folder="/catkin_ws/src/pavilhao_conhecimento/media/sounds/"
sound_path=$HOME$folder$sound

aplay -q $sound_path &

exit 0
