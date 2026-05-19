"""
ROS2 Camera Broadcaster
Jetson CSI Camera (Arducam) -> /camera/image_raw
"""

import cv2
import time

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):

    def __init__(self):

        super().__init__("camera_publisher")

        self.bridge = CvBridge()

        # ROS2 publisher
        self.image_pub = self.create_publisher(
            Image,
            "/camera/image_raw",
            10
        )

        self.cap = None

        self._init_camera()

    # -------------------------------------------------
    # GStreamer pipeline
    # -------------------------------------------------

    def _gstreamer_pipeline(self):

        return (
            "nvarguscamerasrc sensor-id=0 ! "
            "video/x-raw(memory:NVMM), width=1920, height=1080, framerate=30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=960, height=540, format=BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=BGR ! "
            "appsink drop=true max-buffers=1"
        )

    # -------------------------------------------------

    def _init_camera(self):

        time.sleep(0.5)

        pipeline = self._gstreamer_pipeline()

        self.cap = cv2.VideoCapture(
            pipeline,
            cv2.CAP_GSTREAMER
        )

        if not self.cap.isOpened():

            raise RuntimeError(
                "[CAMERA] Failed to open CSI camera"
            )

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        print("[CAMERA] ✓ Camera initialized")

    # -------------------------------------------------

    def run(self):

        print("[CAMERA] Broadcasting ROS2 video feed...")

        while rclpy.ok():

            ret, frame = self.cap.read()

            if not ret:
                time.sleep(0.01)
                continue

            # Convert OpenCV frame -> ROS2 Image message
            msg = self.bridge.cv2_to_imgmsg(
                frame,
                encoding="bgr8"
            )

            msg.header.stamp = (
                self.get_clock()
                .now()
                .to_msg()
            )

            msg.header.frame_id = "camera"

            # Publish
            self.image_pub.publish(msg)

            # Optional preview window
            cv2.imshow("Camera Feed", frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.shutdown()

    # -------------------------------------------------

    def shutdown(self):

        if self.cap:
            self.cap.release()

        cv2.destroyAllWindows()

        print("[CAMERA] Shutdown complete")


# -------------------------------------------------
# MAIN
# -------------------------------------------------

def main():

    rclpy.init()

    node = CameraPublisher()

    try:

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.shutdown()

        node.destroy_node()

        rclpy.shutdown()


if __name__ == "__main__":

    main()
