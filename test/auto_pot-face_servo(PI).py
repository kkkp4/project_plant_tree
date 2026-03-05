import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
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

        # ===== ROS Publisher =====
        self.publisher = self.create_publisher(Image, '/image', 10)

        # ===== ROS Subscriber (รับคำสั่งจาก PC) =====
        self.cmd_sub = self.create_subscription(
            String,
            '/cmd',
            self.cmd_callback,
            10
        )

        # ===== เปิดกล้อง =====
        self.cap = cv2.VideoCapture(CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("❌ เปิดกล้องไม่ได้")
            exit()

        self.get_logger().info("📷 Camera Started")

        # ===== เปิด Serial ไป ESP32 =====
        self.ser = serial.Serial(ESP_PORT, BAUD, timeout=1)
        self.get_logger().info("🔌 ESP32 Connected")

        # ===== SERVO SCAN =====
        self.servo_angle = 180
        self.scan_min = 90
        self.scan_step = 5
        self.detected = False

        # ===== TIMER =====
        self.timer = self.create_timer(0.03, self.publish_image)
        self.scan_timer = self.create_timer(0.5, self.servo_scan)

        # ===== KEYBOARD THREAD =====
        threading.Thread(target=self.keyboard_loop, daemon=True).start()

    # ---------- ส่งภาพ ----------
    def publish_image(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.resize(frame, (RESIZE_WIDTH, RESIZE_HEIGHT))
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')

        self.publisher.publish(msg)

    # ---------- รับคำสั่งจาก PC ----------
    def cmd_callback(self, msg):

        cmd = msg.data
        direction = cmd[0]

        # ถ้าเจอ pot_face
        if direction in ['F', 'L', 'R']:
            self.detected = True

        # ถ้าไม่เจอ
        elif direction == 'S':
            self.detected = False

        # ส่งคำสั่งไป ESP32
        self.ser.write((cmd + "\n").encode())

    # ---------- Servo Scan ----------
    def servo_scan(self):

        if self.detected:
            return

        if self.servo_angle > self.scan_min:

            self.servo_angle -= self.scan_step

            cmd = f"V:{self.servo_angle}"
            self.ser.write((cmd + "\n").encode())

            print("Servo scanning:", self.servo_angle)

    # ---------- Keyboard Control ----------
    def keyboard_loop(self):

        print("\nControl Keys:")
        print("W=Forward | S=Backward | A=Left | D=Right | SPACE=Stop")
        print("1=V:0 | 2=V:45 | 3=V:90 | 4=V:135 | 5=V:180")
        print("Q=Quit\n")

        while rclpy.ok():

            key = self.get_key()

            if key == 'w':
                cmd = "F:80"
            elif key == 's':
                cmd = "B:120"
            elif key == 'a':
                cmd = "L:120"
            elif key == 'd':
                cmd = "R:120"
            elif key == ' ':
                cmd = "S:0"

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

    # ---------- อ่านปุ่ม realtime ----------
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
