import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import time


class PotFaceDetector(Node):

    def __init__(self):
        super().__init__('top_pot_detector')

        self.bridge = CvBridge()
        self.model = YOLO("top_pot3.pt")
        self.names = self.model.names

        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(String, '/cmd', 10)

        # ===== PID =====
        self.Kp = 0.45
        self.Ki = 0.001
        self.Kd = 0.25

        self.prev_error = 0
        self.integral = 0
        self.prev_time = time.time()

        self.max_speed = 12
        self.min_speed = 90
        self.forward_speed = 60

        self.stop_area = 100000
        self.center_deadband = 25

        # ===== IMAGE TIMEOUT =====
        self.last_image_time = time.time()
        self.timer = self.create_timer(0.5, self.check_camera_timeout)

        # ===== START COMMAND =====
        start_msg = String()
        start_msg.data = "V:100"
        self.cmd_pub.publish(start_msg)

        # ===== 2 MIN SHUTDOWN =====
        self.start_time = time.time()

        self.get_logger().info("🚀 System Start → Send V:100")
        self.get_logger().info("✅ Top Pot Auto Align Started")


    # =========================================
    # CAMERA TIMEOUT CHECK
    # =========================================
    def check_camera_timeout(self):

        # ===== 2 MIN SHUTDOWN =====
        if time.time() - self.start_time > 30:

            stop_msg = String()
            stop_msg.data = "S:0"
            self.cmd_pub.publish(stop_msg)

            self.get_logger().warn("⏰ 2 Minutes Passed → Shutdown Node")

            time.sleep(0.3)

            self.destroy_node()
            rclpy.shutdown()
            return

        if time.time() - self.last_image_time > 1.0:

            stop_msg = String()
            stop_msg.data = "S:0"
            self.cmd_pub.publish(stop_msg)

            self.get_logger().warn("⚠️ No image received → STOP")


    # =========================================
    # IMAGE CALLBACK
    # =========================================
    def image_callback(self, msg):

        self.last_image_time = time.time()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame, conf=0.5)

        cmd = "S:0"
        boxes = results[0].boxes

        # =============================
        # CABBAGE SAFETY STOP
        # =============================
        if boxes is not None and len(boxes) > 0:

            cabbage_found = 0

            for box in boxes:

                cls_id = int(box.cls[0])
                label = self.names[cls_id]

                if label == "cabbage":
                    cabbage_found += 1

            if cabbage_found >= 2:

                stop_msg = String()
                stop_msg.data = "S:0"
                self.cmd_pub.publish(stop_msg)

                self.get_logger().warn("🛑 2 Cabbages detected → STOP & EXIT")

                cv2.imshow("Top Pot Auto Align", frame)
                cv2.waitKey(1)

                time.sleep(0.3)

                self.destroy_node()
                rclpy.shutdown()
                return

        # =============================
        # NORMAL DETECTION
        # =============================
        if boxes is not None and len(boxes) > 0:

            centers = []
            areas = []

            for box in boxes:

                cls_id = int(box.cls[0])
                label = self.names[cls_id]

                if label != "top_pot":
                    continue

                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                centers.append((cx, cy))
                areas.append((x2-x1)*(y2-y1))

                cv2.rectangle(frame,(x1,y1),(x2,y2),(0,255,0),2)
                cv2.circle(frame,(int(cx),int(cy)),4,(0,0,255),-1)

            if len(centers) > 0:

                frame_center = frame.shape[1] / 2

                if len(centers) == 1:

                    object_center = centers[0][0]
                    area = areas[0]

                else:

                    centers_sorted = sorted(
                        list(zip(centers,areas)),
                        key=lambda c: abs(c[0][0]-frame_center)
                    )

                    c1, a1 = centers_sorted[0]
                    c2, a2 = centers_sorted[1]

                    object_center = (c1[0] + c2[0]) / 2
                    area = (a1 + a2) / 2

                    cv2.circle(
                        frame,
                        (int(object_center), int((c1[1]+c2[1])/2)),
                        6,
                        (255,0,255),
                        -1
                    )

                # ===== PID =====
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

                    if abs(error) > self.center_deadband:

                        if output > 0:
                            cmd = f"R:{int(abs(output))}"
                        else:
                            cmd = f"L:{int(abs(output)+10)}"

                    else:

                        cmd = f"F:{self.forward_speed}"

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
