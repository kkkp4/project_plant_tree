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

        self.init_timer = self.create_timer(0.6, self.init_servo)
        self.bridge = CvBridge()
        self.model = YOLO("pot_face.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        # ===== ตรวจภาพ =====
        self.last_image_time = time.time()
        self.image_timeout = 0.6
        self.timer = self.create_timer(0.1, self.check_camera)

        # ===== PID =====
        self.Kp = 0.40
        self.Ki = 0.0010
        self.Kd = 0.09

        self.previous_error = 0
        self.integral = 0
        self.previous_time = time.time()

        # ===== SPEED =====
        self.max_speed = 150
        self.min_speed = 100
        self.stop_area = 40000

        # ===== SERVO SCAN =====
        self.servo_angle = 180
        self.min_servo = 114
        self.scan_step = 3
        self.last_scan_time = time.time()

        # ===== STATE =====
        self.forward_sent = False

        self.get_logger().info("✅ Pot Face Detector + Servo Scan")

    def init_servo(self):

        msg = String()
        msg.data = "V:180"
        self.cmd_pub.publish(msg)

        self.get_logger().info("🔧 Servo Init -> 180")

        self.init_timer.cancel()
    # =========================
    # CAMERA CHECK
    # =========================
    def check_camera(self):

        now = time.time()

        if now - self.last_image_time > self.image_timeout:

            stop_msg = String()
            stop_msg.data = "S:0"
            self.cmd_pub.publish(stop_msg)

            self.get_logger().warn("⚠ No Image → STOP")


    # =========================
    # IMAGE CALLBACK
    # =========================
    def image_callback(self, msg):

        self.last_image_time = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=0.5)

        cmd = "S:0"
        cmd_msg = String()

        # =========================================
        # DETECT POT
        # =========================================
        if len(results[0].boxes) > 0:

            box = results[0].boxes[0]
            x1, y1, x2, y2 = box.xyxy[0]

            frame_center = 160
            object_center = (x1 + x2) / 2
            error = float(object_center - frame_center)

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

            output = max(min(output, self.max_speed), -self.max_speed)

            if 0 < abs(output) < self.min_speed:
                output = self.min_speed if output > 0 else -self.min_speed

            area = (x2 - x1) * (y2 - y1)

            # ===== ถึงกระถาง =====
            if area > self.stop_area:

                stop_msg = String()
                stop_msg.data = "S:0"
                self.cmd_pub.publish(stop_msg)

                return

            elif abs(error) < 15:

                cmd = f"F:{self.max_speed}"

            elif output > 0:

                cmd = f"R:{int(abs(output))}"

            else:

                cmd = f"L:{int(abs(output))}"

            self.get_logger().info(
                f"err={int(error)} | out={int(output)} | area={int(area)}"
            )


        # =========================================
        # NOT FOUND POT
        # =========================================
        else:

            self.integral = 0
            cmd = "S:0"

            now = time.time()

            if now - self.last_scan_time > 1.5:

                # ===== servo scan =====
                if self.servo_angle > self.min_servo:

                    self.servo_angle -= self.scan_step

                    servo_cmd = String()
                    servo_cmd.data = f"V:{self.servo_angle}"
                    self.cmd_pub.publish(servo_cmd)

                    self.get_logger().info(
                        f"Scanning Servo -> {self.servo_angle}"
                    )
                elif self.servo_angle == self.min_servo:
                    self.destroy_node()
                    rclpy.shutdown()
                    return

                # ===== scan ครบแล้ว =====
                elif not self.forward_sent:

                    move_cmd = String()
                    move_cmd.data = "D:15"
                    self.cmd_pub.publish(move_cmd)

                    self.get_logger().info(
                        "❌ No object found → Move Forward 15 cm"
                    )

                    self.forward_sent = True

                    time.sleep(0.5)

                    self.get_logger().info("🛑 Shutdown Node")

                    self.destroy_node()
                    rclpy.shutdown()
                    return

                self.last_scan_time = now


        cmd_msg.data = cmd
        self.cmd_pub.publish(cmd_msg)

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
