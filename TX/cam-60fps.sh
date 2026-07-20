#!/bin/bash
# 推流前跑這個:設 1080p + 手動曝光 -> 穩 60fps。可調 EXP(預設16000,<=16600)、GAIN(預設24,1-64補亮)
CAM_MODE=1080p STREAM_ENABLE=0 bash ~/cam-pipeline.sh >/dev/null 2>&1
sleep 1; pkill -x gst-launch-1.0 2>/dev/null
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_mode=0,exposure=${EXP:-16000},gain=${GAIN:-10}
echo "[cam-60fps] 1080p manual exposure=${EXP:-16000} gain=${GAIN:-10} -> 60fps ready"
