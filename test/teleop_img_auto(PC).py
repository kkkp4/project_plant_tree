import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import time


class PotFaceDetector(Node):

    def __init__(self):
        super().__init__('pot_face_detector')

        self.bridge = CvBridge()
        self.model = YOLO("pot_face.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)

        # ===== PID (สำหรับ 320x240) =====
        self.Kp = 0.5
        self.Ki = 0.0008
        self.Kd = 0.15

        self.previous_error = 0
        self.integral = 0
        self.previous_time = time.time()

        # ===== SPEED LIMIT =====
        self.max_speed = 90
        self.min_speed = 30
        self.stop_area = 40000

        self.get_logger().info("✅ PID 320x240 | Speed 30-90")

    def image_callback(self, msg):

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=0.5)

        cmd = "S:0"
        cmd_msg = String()

        if len(results[0].boxes) > 0:

            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0]

            # ===== ERROR =====
            frame_center = 160  # 320/2
            object_center = (x1 + x2) / 2
            error = float(object_center - frame_center)

            # ===== TIME =====
            current_time = time.time()
            dt = current_time - self.previous_time
            if dt == 0:
                dt = 0.0001

            # ===== PID =====
            self.integral += error * dt
            derivative = (error - self.previous_error) / dt

            output = (
                self.Kp * error +
                self.Ki * self.integral +
                self.Kd * derivative
            )

            self.previous_error = error
            self.previous_time = current_time

            # จำกัดความเร็วสูงสุด
            output = max(min(output, self.max_speed), -self.max_speed)

            # ถ้ามีการหมุน ต้องไม่ต่ำกว่า min_speed
            if 0 < abs(output) < self.min_speed:
                output = self.min_speed if output > 0 else -self.min_speed

            # ===== ระยะจาก area =====
            area = (x2 - x1) * (y2 - y1)

            if area > self.stop_area:
                cmd = "S:0"

            elif abs(error) < 15:
                # เดินหน้าใช้ max_speed (ไม่น้อยกว่า min อยู่แล้ว)
                cmd = f"F:{self.max_speed}"

            elif output > 0:
                cmd = f"R:{int(abs(output))}"

            else:
                cmd = f"L:{int(abs(output))}"

            self.get_logger().info(
                f"err={int(error)} | out={int(output)} | area={int(area)}"
            )

        else:
            # ไม่เห็นวัตถุ รีเซ็ต I
            self.integral = 0
            cmd = "S:0"

        cmd_msg.data = cmd
        self.cmd_publisher.publish(cmd_msg)

        annotated_frame = results[0].plot()
        cv2.imshow("Pot Face Detection", annotated_frame)
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
