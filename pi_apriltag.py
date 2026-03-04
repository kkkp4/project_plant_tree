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

        # Subscribe กล้อง
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        # Publisher ส่งไป Pi
        self.publisher = self.create_publisher(
            String,
            '/tag_data',
            10
        )

        # AprilTag Detector
        self.detector = Detector(
            families='tagStandard52h13',
            nthreads=1,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
            debug=0
        )

        self.last_time = time.time()

    def image_callback(self, msg):

        # ตรวจทุก 5 วินาที
        if time.time() - self.last_time < 5:
            return

        self.last_time = time.time()

        # ROS Image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = self.detector.detect(gray)

        if len(detections) == 0:
            self.get_logger().info("No AprilTag detected")
            return

        self.get_logger().info(f"Found {len(detections)} tag(s)")

        for tag in detections:

            tag_value = tag.tag_id

            # บังคับให้เป็น 5 หลักเสมอ
            tag_str = f"{tag_value:05d}"

            first_two = int(tag_str[:2])
            middle = int(tag_str[2])
            last_two = int(tag_str[3:])

            self.get_logger().info(
                f"Tag: {tag_str} -> {first_two} | {middle} | {last_two}"
            )

            # เตรียมข้อมูลส่งไป Pi
            data_dict = {
                "first_two": first_two,
                "middle": middle,
                "last_two": last_two
            }

            msg_out = String()
            msg_out.data = json.dumps(data_dict)

            self.publisher.publish(msg_out)
            self.get_logger().info("Data sent to Pi")

            # วาดกรอบแสดงผล
            corners = tag.corners
            for i in range(4):
                pt1 = tuple(corners[i].astype(int))
                pt2 = tuple(corners[(i+1) % 4].astype(int))
                cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

            center = tuple(tag.center.astype(int))
            cv2.putText(
                frame,
                f"ID: {tag_str}",
                center,
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        cv2.imshow("AprilTag Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
