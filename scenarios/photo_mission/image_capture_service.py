#!/usr/bin/env python3
"""카메라 이미지 한 장을 PNG로 저장하는 ROS 2 서비스."""
import math
import threading
from datetime import datetime
from pathlib import Path
from uuid import uuid4

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


class ImageCaptureService(Node):
    def __init__(self):
        super().__init__("image_capture_service")

        self.declare_parameter(
            "image_topic", "/TurtleBot3Burger/front_camera/image_color"
        )
        self.declare_parameter("save_dir", str(Path.home() / "robot_photos"))
        self.declare_parameter("capture_timeout", 3.0)

        self.image_topic = self.get_parameter("image_topic").value
        self.save_dir = Path(self.get_parameter("save_dir").value).expanduser().resolve()
        self.capture_timeout = float(self.get_parameter("capture_timeout").value)
        if not math.isfinite(self.capture_timeout) or self.capture_timeout <= 0:
            raise ValueError("capture_timeout must be a positive, finite number")
        self.save_dir.mkdir(parents=True, exist_ok=True)

        self.bridge = CvBridge()
        self._condition = threading.Condition()
        self._latest_image = None
        self._frame_count = 0

        # 사진 요청이 기다리는 동안에도 카메라 콜백이 실행되도록 그룹을 분리한다.
        self._image_group = MutuallyExclusiveCallbackGroup()
        self._service_group = MutuallyExclusiveCallbackGroup()

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self._on_image,
            qos_profile_sensor_data,
            callback_group=self._image_group,
        )
        self.capture_service = self.create_service(
            Trigger,
            "/capture_image",
            self._capture_image,
            callback_group=self._service_group,
        )

        self.get_logger().info(f"Camera topic: {self.image_topic}")
        self.get_logger().info(f"Save directory: {self.save_dir}")
        self.get_logger().info("Capture service ready: /capture_image")

    def _on_image(self, msg):
        with self._condition:
            self._latest_image = msg
            self._frame_count += 1
            first_frame = self._frame_count == 1
            self._condition.notify_all()

        if first_frame:
            self.get_logger().info(
                f"First camera frame: {msg.width}x{msg.height}, encoding={msg.encoding}"
            )

    def _capture_image(self, _request, response):
        file_path = None
        try:
            # 이전에 캐시한 이미지 대신, 요청 이후 새로 수신한 프레임을 기다린다.
            with self._condition:
                previous_count = self._frame_count
                received = self._condition.wait_for(
                    lambda: self._frame_count > previous_count,
                    timeout=self.capture_timeout,
                )
                if not received:
                    raise TimeoutError(
                        f"No new camera frame within {self.capture_timeout:.1f}s"
                    )
                msg = self._latest_image

            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            if image is None or image.size == 0:
                raise ValueError("Received an empty image")

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            file_path = self.save_dir / f"capture_{timestamp}_{uuid4().hex[:8]}.png"

            if not cv2.imwrite(str(file_path), image):
                raise OSError("cv2.imwrite failed")
            if not file_path.is_file() or file_path.stat().st_size == 0:
                raise OSError("The saved image file is missing or empty")

            response.success = True
            response.message = str(file_path)
            self.get_logger().info(f"Saved image: {file_path}")

        except Exception as error:
            if file_path is not None:
                try:
                    file_path.unlink(missing_ok=True)
                except OSError:
                    pass
            response.success = False
            response.message = f"Capture failed: {error}"
            self.get_logger().error(response.message)

        return response


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = ImageCaptureService()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
