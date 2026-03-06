import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np
import time


class PotFaceDetector(Node):

    def __init__(self):
        super().__init__('pot_face_detector')

        self.bridge = CvBridge()
        self.model = YOLO("pot_face.pt")

        # รับภาพ
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        # ส่งคำสั่งไป ESP32
        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        # ===== PID =====
        self.Kp = 0.6
        self.Ki = 0.0008
        self.Kd = 0.2

        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

        # ===== SPEED =====
        self.max_speed = 90
        self.min_speed = 30

        # ===== STOP DISTANCE =====
        self.stop_area = 45000

        self.get_logger().info("✅ Pot Auto Align Started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, conf=0.5)

        cmd = "S:0"

        if len(results[0].boxes) > 0:

            box = results[0].boxes[0]

            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # ===== ROI =====
            roi = frame[y1:y2, x1:x2]

            if roi.size == 0:
                return

            gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
            _, thresh = cv2.threshold(gray, 80, 255, cv2.THRESH_BINARY)

            contours, _ = cv2.findContours(
                thresh,
                cv2.RETR_EXTERNAL,
                cv2.CHAIN_APPROX_SIMPLE
            )

            if len(contours) == 0:
                return

            cnt = max(contours, key=cv2.contourArea)

            # ===== ROTATED RECT =====
            rect = cv2.minAreaRect(cnt)
            angle = rect[2]

            box_points = cv2.boxPoints(rect)
            box_points = np.int0(box_points)

            box_points[:, 0] += x1
            box_points[:, 1] += y1

            cv2.drawContours(frame, [box_points], 0, (0, 255, 0), 2)

            # ===== CENTER ERROR =====
            frame_center = frame.shape[1] / 2
            object_center = (x1 + x2) / 2

            error = object_center - frame_center

            # ===== TIME =====
            now = time.time()
            dt = now - self.prev_time
            if dt == 0:
                dt = 0.0001

            # ===== PID =====
            self.integral += error * dt
            derivative = (error - self.prev_error) / dt

            output = (
                self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative
            )

            self.prev_error = error
            self.prev_time = now

            # ===== LIMIT SPEED =====
            output = max(min(output, self.max_speed), -self.max_speed)

            if 0 < abs(output) < self.min_speed:
                output = self.min_speed if output > 0 else -self.min_speed

            # ===== DISTANCE =====
            area = (x2 - x1) * (y2 - y1)

            # ===== DECISION =====
            if area > self.stop_area:
                cmd = "S:0"

            else:

                # หมุนตาม angle ก่อน
                if angle > 5:
                    cmd = f"R:{self.min_speed}"

                elif angle < -5:
                    cmd = f"L:{self.min_speed}"

                else:

                    # ใช้ PID จัดกลาง
                    if abs(error) < 20:
                        cmd = f"F:{self.max_speed}"

                    elif output > 0:
                        cmd = f"R:{int(abs(output))}"

                    else:
                        cmd = f"L:{int(abs(output))}"

            # ===== LOG =====
            self.get_logger().info(
                f"angle={angle:.2f} | err={int(error)} | area={int(area)} | cmd={cmd}"
            )

        else:

            self.integral = 0
            cmd = "S:0"

        # ===== SEND CMD =====
        msg_cmd = String()
        msg_cmd.data = cmd
        self.cmd_pub.publish(msg_cmd)

        annotated = results[0].plot()
        cv2.imshow("Pot Auto Align", annotated)
        cv2.waitKey(1)


def main(args=None):

    rclpy.init(args=args)

    node = PotFaceDetector()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
