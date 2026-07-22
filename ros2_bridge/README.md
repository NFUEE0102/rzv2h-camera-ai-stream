# rzv2h_detection_bridge

A standalone ROS 2 node that republishes this repo's existing UDP-JSON
detection sidecar (`bbox_udp`, UDP port 50012, board → PC — see the main
repo README's "Wire protocol" section) as standard `vision_msgs` topics.

## Why this exists

The core TX/RX pipeline embeds detections directly in the H.265 stream as
SEI data — that's the primary path, and it's deliberately not ROS 2 (no
DDS overhead, no separate channel to fall out of sync with the video). But
`app_m5` already *also* sends a plain UDP-JSON copy of every detection as a
fallback for viewers that can't parse SEI. This package just adds a second
consumer of that already-existing feed, so anything in a ROS 2 graph
(rviz2, Foxglove, a flight-control node subscribing over micro-ROS, etc.)
can pick up detections without touching the proven core pipeline at all.

No changes to `TX/` or `RX/` are required to use this.

## Topics published

| Topic | Type | Notes |
|---|---|---|
| `~/detections` | `vision_msgs/Detection2DArray` | One `Detection2D` per box; `bbox.center` is the box center in source-frame pixels (matches `bbox_udp`'s `x`/`y`), `size_x`/`size_y` are `w`/`h`. `results[0].hypothesis.class_id` is the class id (as a string), `.score` is the confidence (0-1). |
| `~/imu_roll_deg` | `std_msgs/Float32` | IMU roll in degrees. Only published for packets where `bbox_udp` marked the reading valid (i.e. the `"roll"` field was present) — no stale/fabricated values on a dead IMU. |

## Parameters

| Parameter | Default | Notes |
|---|---|---|
| `udp_port` | `50012` | Must match `BBOX_PORT` in the main repo's `link_proto.py` / `STREAM_BBOX_PORT` in `tx_config.py`. |
| `frame_id` | `rzv2h_camera` | Frame id stamped on published messages. |

## Building

Standard `ament_python` package — drop this directory into a ROS 2
workspace's `src/` and build normally:

```bash
colcon build --packages-select rzv2h_detection_bridge
source install/setup.bash
ros2 run rzv2h_detection_bridge bridge_node --ros-args -p udp_port:=50012
```

## Where this runs

This listens for UDP packets, so run it wherever the board's `bbox_udp`
traffic actually lands — that's the ground-station machine by default (the
same destination the RX viewer runs on), since `app_m5` sends the sidecar
to the discovered ground-station IP alongside the RTP video stream.

If you want detections consumed *on the board itself* (e.g. by an R8/CM33
flight-control task over RPMsg/OpenAMP), `app_m5` would need a second
`bbox_udp_init()` destination pointed at `127.0.0.1` — that's a small
change to `TX/engine_app_src/main_yolox.cpp` / `tx_config.py`, not yet
made, since it wasn't clear this was needed versus the ground-station case
above. Flagging it here rather than guessing at a change to the core
pipeline.

## Status

Written against the `vision_msgs`/`rclpy` APIs as documented for ROS 2
Jazzy. **Not yet run against a real ROS 2 install or the real `bbox_udp`
feed** — there was no ROS 2 environment or working camera available at
the time this was written to verify end-to-end. Treat it as a reasonable
starting point, not a tested component, until it's actually been run.
