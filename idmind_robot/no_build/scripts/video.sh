#!/bin/bash

proj=$2
video=$1
folder="/catkin_ws/src/pavilhao_conhecimento/media/videos/"
video_path=$HOME$folder$video

vlc --fullscreen --quiet $video_path &
#sleep 1
#wmctrl -r "VLC" -b toggle,fullscreen

duration=$(mediainfo --Inform="General;%Duration%" $video_path)
seconds=$((duration / 1000))
rounded=$(printf %.0f $seconds)

sleep $rounded
#sleep 30

#wmctrl -c $video
kill $(pidof vlc)
#wmctrl -c "VLC"

exit 0
