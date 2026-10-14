"""YOLO11n (TensorRT) + ByteTrack on the colour camera; publishes tracked objects as JSON."""
from __future__ import annotations

import json
import math
import os
import time

import numpy as np
import rclpy
from hwr_core.navigation.vision_geometry import bearing, object_range
from hwr_sensors.depth_to_ranges_node import _depth_metres
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

ENGINE = os.environ.get("HWR_YOLO_ENGINE", "/home/ubuntu/third_party/yolo11n.engine")


class VisionNode(Node):
    def __init__(self):
        super().__init__("hwr_vision")
        from ultralytics import YOLO
        self.model = YOLO(ENGINE, task="detect")
        self.depth = None
        self.fx = self.cx = None
        self.last = 0.0
        qos = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(String, "vision/objects", 1)
        self.create_subscription(Image, "camera/color/image_raw", self._color, qos)
        self.create_subscription(Image, "camera/depth/image_raw", self._depth, qos)
        self.create_subscription(CameraInfo, "camera/color/camera_info", self._info, qos)
        self.get_logger().info(f"vision ready: {ENGINE}")

    def _info(self, msg: CameraInfo) -> None:
        self.fx, self.cx = msg.k[0], msg.k[2]

    def _depth(self, msg: Image) -> None:
        try:
            self.depth = _depth_metres(msg)
        except ValueError:
            pass

    def _color(self, msg: Image) -> None:
        now = time.monotonic()
        if now - self.last < 0.1 or msg.encoding not in ("rgb8", "bgr8"):
            return
        self.last = now
        image = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[
            :, :msg.width * 3].reshape(msg.height, msg.width, 3)
        if msg.encoding == "rgb8":
            image = image[:, :, ::-1]
        started = time.monotonic()
        result = self.model.track(image, persist=True, tracker="bytetrack.yaml",
                                  conf=0.35, verbose=False)[0]
        objects = []
        for box in result.boxes:
            x1, y1, x2, y2 = (float(v) for v in box.xyxy[0])
            item = {"class": result.names[int(box.cls)], "conf": round(float(box.conf), 2),
                    "id": int(box.id) if box.id is not None else None,
                    "box": [round(x1), round(y1), round(x2), round(y2)]}
            if self.fx:
                item["bearing_deg"] = round(math.degrees(bearing((x1 + x2) / 2, self.fx, self.cx)))
            if self.depth is not None:
                item["range_m"] = object_range(self.depth, (x1, y1, x2, y2), image.shape[:2])
            objects.append(item)
        self.pub.publish(String(data=json.dumps({
            "stamp": time.time(), "infer_ms": round((time.monotonic() - started) * 1000),
            "objects": objects})))


def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
