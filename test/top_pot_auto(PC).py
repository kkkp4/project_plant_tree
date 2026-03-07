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
        super().__init__('top_pot_detector')

        self.bridge = CvBridge()
        self.model = YOLO("top_pot.pt")   # เปลี่ยนโมเดล

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
        self.Kp = 0.50
        self.Ki = 0.0015
        self.Kd = 0.23

        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

        # ===== SPEED =====
        self.max_speed = 75
        self.min_speed = 35

        # ===== STOP DISTANCE =====
        self.stop_area = 80000

        self.get_logger().info("✅ Top Pot Auto Align Started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, conf=0.5)

        cmd = "S:0"

        if len(results[0].boxes) > 0:

            box = results[0].boxes[0]

            x1, y1, x2, y2 = box.xyxy[0]
            x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

            # ===== CENTER =====
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

                if abs(error) < 20:

                    cmd = f"F:{self.max_speed}"

                elif output > 0:

                    cmd = f"R:{int(abs(output))}"

                else:

                    cmd = f"L:{int(abs(output))}"

            # ===== VISUAL =====
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

            cx = int(object_center)
            cy = int((y1 + y2)/2)

            cv2.circle(frame, (cx,cy), 5, (0,0,255), -1)

            cv2.line(
                frame,
                (int(frame_center),0),
                (int(frame_center),frame.shape[0]),
                (255,255,0),
                2
            )

            # ===== LOG =====
            self.get_logger().info(
                f"err={int(error)} | area={int(area)} | cmd={cmd}"
            )

        else:

            self.integral = 0
            cmd = "S:0"

        # ===== SEND CMD =====
        msg_cmd = String()
        msg_cmd.data = cmd
        self.cmd_pub.publish(msg_cmd)

        cv2.imshow("Top Pot Auto Align", frame)
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
