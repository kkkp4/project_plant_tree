import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import serial
import threading
import sys
import termios
import tty

# -------- CONFIG --------
ESP_PORT = "/dev/ttyUSB0"
BAUD = 115200
CAM_INDEX = 0
RESIZE_WIDTH = 320
RESIZE_HEIGHT = 240
# ------------------------

class PiCameraMotor(Node):

    def __init__(self):
        super().__init__('pi_camera_motor')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/image', 10)

        # เปิดกล้อง
        self.cap = cv2.VideoCapture(CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("❌ เปิดกล้องไม่ได้")
            exit()

        self.get_logger().info("📷 Camera Started")

        # เปิด Serial ไป ESP32
        self.ser = serial.Serial(ESP_PORT, BAUD, timeout=1)
        self.get_logger().info("🔌 ESP32 Connected")
        
        # ส่งภาพ ~30 FPS
        self.timer = self.create_timer(0.03, self.publish_image)

        # Thread อ่านคีย์บอร์ด
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    # ---------- ส่งภาพ ----------
    def publish_image(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(msg)

    # ---------- ควบคุมมอเตอร์ + Servo ----------
    def keyboard_loop(self):
        print("\nControl Keys:")
        print("W=Forward | S=Backward | A=Left | D=Right | SPACE=Stop")
        print("1=V:0 | 2=V:45 | 3=V:90 | 4=V:135 | 5=V:180")
        print("Q=Quit\n")

        while rclpy.ok():
            key = self.get_key()

            # ---------- Motor ----------
            if key == 'w':
                cmd = "F:30"
            elif key == 's':
                cmd = "B:120"
            elif key == 'a':
                cmd = "L:120"
            elif key == 'd':
                cmd = "R:120"
            elif key == ' ':
                cmd = "S:0"

            # ---------- Servo ----------
            elif key == '1':
                cmd = "V:0"
            elif key == '2':
                cmd = "V:45"
            elif key == '3':
                cmd = "V:90"
            elif key == '4':
                cmd = "V:135"
            elif key == '5':
                cmd = "V:180"

            elif key == 'q':
                print("Exiting...")
                rclpy.shutdown()
                break
            else:
                cmd = None

            if cmd:
                self.ser.write((cmd + "\n").encode())
                print("Sent:", cmd)

    # ---------- อ่านปุ่มแบบ realtime ----------
    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


def main(args=None):
    rclpy.init(args=args)
    node = PiCameraMotor()
    rclpy.spin(node)

    node.cap.release()
    node.ser.close()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
