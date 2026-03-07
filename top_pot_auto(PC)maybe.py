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
        self.model = YOLO("top_pot2.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        # PID
        self.Kp = 0.50
        self.Ki = 0.0015
        self.Kd = 0.23

        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

        self.max_speed = 75
        self.min_speed = 35

        self.stop_area = 80000

        self.get_logger().info("✅ Top Pot Auto Align Started")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame, conf=0.5)

        cmd = "S:0"

        boxes = results[0].boxes

        if boxes is not None and len(boxes) > 0:

            centers = []
            areas = []

            for box in boxes:

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                centers.append((cx, cy))
                areas.append((x2-x1)*(y2-y1))

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.circle(frame,(int(cx),int(cy)),4,(0,0,255),-1)

            # ===== center logic =====

            if len(centers) == 1:

                object_center = centers[0][0]
                area = areas[0]

            else:

                # เลือกสองวัตถุที่ใกล้กลางภาพที่สุด
                frame_center = frame.shape[1]/2

                centers_sorted = sorted(
                    centers,
                    key=lambda c: abs(c[0]-frame_center)
                )

                c1 = centers_sorted[0]
                c2 = centers_sorted[1]

                object_center = (c1[0] + c2[0]) / 2

                # ใช้ area เฉลี่ย
                area = np.mean(areas)

                cv2.circle(frame,(int(object_center),int((c1[1]+c2[1])/2)),6,(255,0,255),-1)

            # ===== PID =====

            frame_center = frame.shape[1] / 2
            error = object_center - frame_center

            now = time.time()
            dt = now - self.prev_time
            if dt == 0:
                dt = 0.0001

            self.integral += error * dt
            derivative = (error - self.prev_error) / dt

            output = (
                self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative
            )

            self.prev_error = error
            self.prev_time = now

            output = max(min(output, self.max_speed), -self.max_speed)

            if 0 < abs(output) < self.min_speed:
                output = self.min_speed if output > 0 else -self.min_speed

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

            cv2.line(
                frame,
                (int(frame_center),0),
                (int(frame_center),frame.shape[0]),
                (255,255,0),
                2
            )

            self.get_logger().info(
                f"err={int(error)} | area={int(area)} | cmd={cmd}"
            )

        else:

            self.integral = 0
            cmd = "S:0"

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
