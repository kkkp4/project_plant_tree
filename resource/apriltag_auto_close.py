import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import time
import json


class AprilTagSubscriber(Node):

    def __init__(self):

        super().__init__('apriltag_subscriber')

        self.bridge = CvBridge()

        # ===== รับภาพ =====
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        # ===== ส่ง tag data =====
        self.tag_publisher = self.create_publisher(
            String,
            '/tag_data',
            10
        )

        # ===== ส่งคำสั่งมอเตอร์ =====
        self.cmd_pub = self.create_publisher(
            String,
            '/cmd',
            10
        )

        # ===== AprilTag Detector =====
        self.detector = Detector(
            families='tagStandard52h13',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        # ===== Camera Watchdog =====
        self.last_image_time = time.time()

        self.timer = self.create_timer(
            0.5,
            self.check_camera_timeout
        )

        self.get_logger().info("🚀 AprilTag detector started")


    # ==========================
    # Camera Timeout
    # ==========================
    def check_camera_timeout(self):

        if time.time() - self.last_image_time > 1.0:

            stop_msg = String()
            stop_msg.data = "S:0"
            self.cmd_pub.publish(stop_msg)

            self.get_logger().warn(
                "⚠️ No image received → STOP"
            )


    # ==========================
    # Image Callback
    # ==========================
    def image_callback(self, msg):

        self.last_image_time = time.time()

        frame = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding='bgr8'
        )

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)

        cmd_msg = String()

        # ======================
        # ไม่เจอ Tag
        # ======================
        if len(detections) == 0:

            cmd_msg.data = "B:80"
            self.cmd_pub.publish(cmd_msg)

            self.get_logger().info(
                "No tag → Moving Backward"
            )

        # ======================
        # เจอ Tag
        # ======================
        else:

            cmd_msg.data = "S:0"
            self.cmd_pub.publish(cmd_msg)

            self.get_logger().info(
                f"Found {len(detections)} tag(s)"
            )

            for tag in detections:

                tag_value = tag.tag_id
                tag_str = f"{tag_value:05d}"

                first_two = int(tag_str[:2])
                middle = int(tag_str[2])
                last_two = int(tag_str[3:])

                self.get_logger().info(
                    f"Tag: {tag_str} -> "
                    f"{first_two} | {middle} | {last_two}"
                )

                # ===== ส่งข้อมูล =====
                data_dict = {
                    "first_two": first_two,
                    "middle": middle,
                    "last_two": last_two
                }

                msg_out = String()
                msg_out.data = json.dumps(data_dict)

                self.tag_publisher.publish(msg_out)

                # ===== Save to file =====
                with open("tag_data.json", "w") as f:
                    json.dump(data_dict, f, indent=4)

                self.get_logger().info("💾 Tag data saved")
                
                self.get_logger().info(
                    "📤 Tag data sent to RobotController"
                )

                # ===== วาดกรอบ =====
                corners = tag.corners

                for i in range(4):

                    pt1 = tuple(
                        corners[i].astype(int)
                    )

                    pt2 = tuple(
                        corners[(i+1)%4].astype(int)
                    )

                    cv2.line(
                        frame,
                        pt1,
                        pt2,
                        (0,255,0),
                        2
                    )

                center = tuple(
                    tag.center.astype(int)
                )

                cv2.putText(
                    frame,
                    f"ID:{tag_str}",
                    center,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0,0,255),
                    2
                )

                # ===== ปิด Node =====
                time.sleep(0.5)

                self.get_logger().info(
                    "🛑 AprilTag detected → Shutdown node"
                )

                self.destroy_node()
                rclpy.shutdown()

                return

        cv2.imshow(
            "AprilTag Detection",
            frame
        )

        cv2.waitKey(1)


# ==========================
# Main
# ==========================
def main(args=None):

    rclpy.init(args=args)

    node = AprilTagSubscriber()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()

    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
