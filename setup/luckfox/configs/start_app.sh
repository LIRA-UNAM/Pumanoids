#!/bin/sh
/oem/usr/bin/RkLunch-stop.sh
export LD_LIBRARY_PATH=/oem/usr/lib:$LD_LIBRARY_PATH
echo "Pumas script initiated"
cd /root/luckfox_pico_rtsp_yolov5_demo
./luckfox_pico_rtsp_yolov5
sleep 1
echo "Pumas node finished. Restarting"

