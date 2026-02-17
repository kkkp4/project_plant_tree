import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from pupil_apriltags import Detector
import time

#ros2 run image_tools cam2image

class AprilTagSubscriber(Node):

    def __init__(self):
        super().__init__('apriltag_subscriber')

        self.bridge = CvBridge()

        # subscribe topic /image
        self.subscription = self.create_subscription(
            Image,
            '/image',
            self.image_callback,
            10
        )

        # สร้าง AprilTag detector
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

        # เช็คทุก 5 วินาที
        if time.time() - self.last_time < 5:
            return

        self.last_time = time.time()

        # แปลง ROS image → OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # ตรวจจับ AprilTag
        detections = self.detector.detect(gray)

        # ถ้าไม่เจอ
        if len(detections) == 0:
            self.get_logger().info("Checked image: No AprilTag detected")

        # ถ้าเจอ
        else:
            self.get_logger().info(f"Checked image: Found {len(detections)} tag(s)")

            for tag in detections:
                self.get_logger().info(f"Detected tag ID: {tag.tag_id}")

                corners = tag.corners
                for i in range(4):
                    pt1 = tuple(corners[i].astype(int))
                    pt2 = tuple(corners[(i+1) % 4].astype(int))
                    cv2.line(frame, pt1, pt2, (0, 255, 0), 2)

                center = tuple(tag.center.astype(int))
                cv2.putText(
                    frame,
                    f"ID: {tag.tag_id}",
                    center,
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

        # แสดงภาพ
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
