#!/usr/bin/env python3
"""
bridge_node.py -- listens on the existing bbox_udp JSON sidecar (UDP,
BBOX_PORT / STREAM_BBOX_PORT = 50012, board -> PC, already sent by every
app_m5 run as a SEI fallback) and republishes detections as standard ROS 2
vision_msgs topics.

This is purely additive: no changes to the TX/RX pipeline are needed. It
just listens to a UDP feed that already exists.
"""
import json
import socket

import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Time
from std_msgs.msg import Float32
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose


class Rzv2hDetectionBridge(Node):
    def __init__(self):
        super().__init__('rzv2h_detection_bridge')

        self.declare_parameter('udp_port', 50012)
        self.declare_parameter('frame_id', 'rzv2h_camera')
        port = self.get_parameter('udp_port').value
        self.frame_id = self.get_parameter('frame_id').value

        self.det_pub = self.create_publisher(Detection2DArray, 'detections', 10)
        self.roll_pub = self.create_publisher(Float32, 'imu_roll_deg', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.sock.bind(('0.0.0.0', port))
        self.get_logger().info(f'listening for bbox_udp JSON on UDP :{port}')

        # Poll the non-blocking socket from a timer so this stays a normal
        # single-threaded rclpy node instead of needing a recv thread.
        self.create_timer(0.005, self._poll)

    def _poll(self):
        while True:
            try:
                data, _addr = self.sock.recvfrom(8192)
            except BlockingIOError:
                return
            except OSError:
                return
            self._handle_packet(data)

    def _handle_packet(self, data: bytes):
        try:
            msg = json.loads(data.decode('utf-8', 'replace'))
        except (json.JSONDecodeError, UnicodeDecodeError):
            self.get_logger().warn(
                'dropped malformed bbox_udp packet', throttle_duration_sec=5.0)
            return

        stamp = self._stamp_from_ts_ms(msg.get('ts_ms', 0))

        arr = Detection2DArray()
        arr.header.stamp = stamp
        arr.header.frame_id = self.frame_id

        for det in msg.get('det', []):
            d = Detection2D()
            d.header = arr.header
            # bbox_udp's x/y are center-based pixel coordinates (see the
            # main repo README's wire-protocol section), matching
            # vision_msgs' bbox.center convention directly.
            d.bbox.center.position.x = float(det.get('x', 0.0))
            d.bbox.center.position.y = float(det.get('y', 0.0))
            d.bbox.size_x = float(det.get('w', 0.0))
            d.bbox.size_y = float(det.get('h', 0.0))

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = str(det.get('c', -1))
            hyp.hypothesis.score = float(det.get('p', 0.0))
            d.results.append(hyp)

            arr.detections.append(d)

        self.det_pub.publish(arr)

        # bbox_udp only emits "roll" when the IMU reading is valid (see
        # bbox_udp.h) -- mirror that here rather than publishing a stale 0.0.
        if 'roll' in msg:
            roll_msg = Float32()
            roll_msg.data = float(msg['roll'])
            self.roll_pub.publish(roll_msg)

    @staticmethod
    def _stamp_from_ts_ms(ts_ms) -> Time:
        t = Time()
        t.sec = int(ts_ms // 1000)
        t.nanosec = int((ts_ms % 1000) * 1_000_000)
        return t

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = Rzv2hDetectionBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
