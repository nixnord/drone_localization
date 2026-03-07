import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests
import threading
import json

# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------
FASTAPI_URL        = "http://127.0.0.1:8000/infer"
INFERENCE_RATE_HZ  = 10       # timer rate — frames sent to FastAPI per second
JPEG_QUALITY       = 85       # JPEG encode quality (0-100)
REQUEST_TIMEOUT_S  = 2.0      # HTTP request timeout in seconds


class FireDetectionClient(Node):

    def __init__(self):
        super().__init__("fire_detection_client")

        # ------------------------------------------------------------------
        # Internal state
        # ------------------------------------------------------------------
        self._bridge       = CvBridge()
        self._latest_frame = None           # latest cached camera frame
        self._lock         = threading.Lock()

        # ------------------------------------------------------------------
        # Subscriber — camera frames from Gazebo bridge
        # Do NOT do inference here — only cache the frame
        # ------------------------------------------------------------------
        self._sub = self.create_subscription(
            Image,
            "/drone/camera/image_raw",
            self._image_callback,
            10
        )

        # ------------------------------------------------------------------
        # Publisher — fire detection results as JSON string
        # ------------------------------------------------------------------
        self._pub = self.create_publisher(
            String,
            "/drone/fire_detection",
            10
        )

        # ------------------------------------------------------------------
        # Timer — drives inference at fixed rate, decoupled from camera rate
        # ------------------------------------------------------------------
        self._timer = self.create_timer(
            1.0 / INFERENCE_RATE_HZ,
            self._send_to_fastapi
        )

        self.get_logger().info(
            f"FireDetectionClient started at {INFERENCE_RATE_HZ} Hz. "
            f"Waiting for frames on /drone/camera/image_raw ..."
        )

    # ------------------------------------------------------------------
    # Camera subscriber callback — cache latest frame only
    # ------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            with self._lock:
                self._latest_frame = frame
        except Exception as e:
            self.get_logger().error(f"cv_bridge conversion failed: {e}")

    # ------------------------------------------------------------------
    # Timer callback — encode and POST to FastAPI
    # ------------------------------------------------------------------
    def _send_to_fastapi(self) -> None:
        # Grab latest frame safely
        with self._lock:
            if self._latest_frame is None:
                return
            frame = self._latest_frame.copy()

        try:
            # Step 1 — encode frame as JPEG in memory
            success, buffer = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY]
            )
            if not success:
                self.get_logger().warn("JPEG encode failed — skipping frame.")
                return

            # Step 2 — base64 encode
            # FastAPI ImageInput expects: {"image": "<base64 string>"}
            # Do NOT prepend "data:image/jpeg;base64," — plain base64 only
            b64_string = base64.b64encode(buffer.tobytes()).decode("utf-8")

            # Step 3 — POST as JSON
            response = requests.post(
                FASTAPI_URL,
                json={"image": b64_string},
                timeout=REQUEST_TIMEOUT_S
            )

            # Step 4 — handle response
            if response.status_code == 200:
                self._handle_response(response.json())
            else:
                self.get_logger().warn(
                    f"FastAPI returned HTTP {response.status_code}"
                )

        except requests.exceptions.Timeout:
            self.get_logger().warn(
                f"FastAPI request timed out (>{REQUEST_TIMEOUT_S}s) — skipping frame."
            )
        except requests.exceptions.ConnectionError:
            self.get_logger().error(
                "Cannot reach FastAPI at 127.0.0.1:8000 — is it running?"
            )
        except Exception as e:
            self.get_logger().error(f"Unexpected error in send_to_fastapi: {e}")

    # ------------------------------------------------------------------
    # Response handler — covers all four states from fire_server.py
    # ------------------------------------------------------------------
    def _handle_response(self, result: dict) -> None:
        status = result.get("status")

        # --- Buffering — server collecting frames for 4-frame temporal stack ---
        if status == "buffering":
            self.get_logger().info(
                f"Buffering frames: {result.get('message', '')}"
            )
            return

        # --- Error — server-side decode or model failure ---
        if status == "error":
            self.get_logger().error(
                f"Server error: {result.get('message', 'unknown')}"
            )
            return

        # --- Success --- fire_detected can be True or False
        if status == "success":
            fire_detected    = result.get("fire_detected", False)
            pixel_count      = result.get("fire_pixel_count", 0)
            coverage_pct     = result.get("fire_coverage_pct", 0.0)
            confidence       = result.get("confidence", 0.0)
            mask_file        = result.get("mask_file", "")

            if fire_detected:
                self.get_logger().warn(
                    f"FIRE DETECTED — "
                    f"coverage={coverage_pct}% | "
                    f"confidence={confidence:.4f} | "
                    f"pixels={pixel_count} | "
                    f"mask={mask_file}"
                )
            else:
                self.get_logger().debug("No fire detected.")

            # Publish full result as JSON string with ROS2 timestamp
            payload = {
                "timestamp":         self.get_clock().now().nanoseconds,
                "fire_detected":     fire_detected,
                "fire_pixel_count":  pixel_count,
                "fire_coverage_pct": coverage_pct,
                "confidence":        confidence,
                "mask_file":         mask_file
            }
            msg = String()
            msg.data = json.dumps(payload)
            self._pub.publish(msg)
            return

        # --- Unknown status --- defensive fallback
        self.get_logger().warn(f"Unknown response status: {status} — full response: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = FireDetectionClient()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()