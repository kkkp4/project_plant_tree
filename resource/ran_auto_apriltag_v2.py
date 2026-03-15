import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import serial
import time
import json

# -------- CONFIG --------
ESP1_PORT = "/dev/ttyUSB0"
ESP2_PORT = "/dev/ttyUSB1"
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

        # รับคำสั่งจาก PC
        self.create_subscription(
            String,
            '/cmd',
            self.cmd_callback,
            10
        )

        # รับ Tag Data
        self.create_subscription(
            String,
            '/tag_data',
            self.tag_callback,
            10
        )

        # เปิดกล้อง
        self.cap = cv2.VideoCapture(CAM_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error("❌ เปิดกล้องไม่ได้")
            exit()

        self.get_logger().info("📷 Camera Started")

        # Serial
        self.esp1 = serial.Serial(ESP1_PORT, BAUD, timeout=1)
        self.esp2 = serial.Serial(ESP2_PORT, BAUD, timeout=1)

        time.sleep(3)

        self.get_logger().info("🔌 ESP32 Connected")

        # tag data
        self.tag_received = False
        self.first_two = None
        self.middle = None
        self.last_two = None

        # ส่งภาพ ~15 FPS
        self.timer = self.create_timer(0.066, self.publish_image)

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
        self.esp1.write((cmd + "\n").encode())
        self.get_logger().info(f"AUTO Sent: {cmd}")

    # =========================
    # รับค่า Tag
    # =========================
    def tag_callback(self, msg):

        if self.tag_received:
            return

        data = json.loads(msg.data)

        self.first_two = data["first_two"]
        self.middle = data["middle"]
        self.last_two = data["last_two"]

        # แปลงค่า middle
        if self.middle == 1:
            self.middle = 5
        elif self.middle == 2:
            self.middle = 10
        elif self.middle == 3:
            self.middle = 15
        elif self.middle == 4:
            self.middle = 20
        elif self.middle == 5:
            self.middle = 25

        self.get_logger().info(
            f"Received: {self.first_two} | {self.middle} | {self.last_two}"
        )

        self.tag_received = True

        time.sleep(1)
        self.esp1.reset_input_buffer()
        self.esp1.setDTR(True)

        self.esp2.setDTR(False)
        time.sleep(1)
        self.esp2.reset_input_buffer()
        self.esp2.setDTR(True)
        self.run_sequence()

    # =========================
    # ส่งคำสั่ง + รอ DONE
    # =========================
    def send_cmd(self, esp, cmd):
        
        esp.reset_input_buffer()

        esp.write((cmd + "\n").encode())

        print("SEND:", cmd)

        while True:

            line = esp.readline().decode(errors="ignore").strip()

            if line != "":
                print("ESP:", line)

            if line == "DONE":
                break

    # =========================
    # sequence ปลูก
    # =========================
    def run_sequence(self):

        x = int(self.first_two)

        sequence = [

            ("esp1", f"D:{x}"),

            # ตั้งขุด
            ("esp2", "SERVO2:155"),
            ("esp1", "D:8"),

            # ลง ขุด 9
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"), 
            ("esp2", "STEP1_FWD:120"),  

            #ขึ้นลง  
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_FWD:120"),  
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_FWD:120"),  
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_FWD:120"),
    

            #ขึ้น9
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),

            # เก็บขุด
            ("esp2", "SERVO2:60"),
            ("esp1", "D:14"),

            #ลง ปล฿ก11
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "STEP1_FWD:120"),

            #เปิดที่ปลูก
            ("esp2", "SERVO1:120"), 


            # กลบ
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "SERVO1:70"),#อ้าออก
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "SERVO1:100"),#ปิด
            ("esp2", "SERVO1:70"),#อ้าออก
            
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "SERVO1:70"),#อ้าออก
            ("esp2", "STEP1_FWD:120"),
            ("esp2", "SERVO1:100"),#ปิด
            ("esp2", "SERVO1:70"),#อ้าออก


            #ขึ้น11
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),
            ("esp2", "STEP1_BWD:80"),

            #ปิดที่ปลูก
            ("esp2", "SERVO1:120"),
            #โหลดต้นกล้า
            ("esp2", "STEP2_45:60"),
        ]

        loops = self.middle

        for r in range(loops):

            print("\n===== ROUND", r+1, "=====")

            for device, cmd in sequence:

                if device == "esp1":
                    self.send_cmd(self.esp1, cmd)

                elif device == "esp2":
                    self.send_cmd(self.esp2, cmd)

                time.sleep(1.5)

        print("FINISH")

    # ---------- ปิดระบบ ----------
    def destroy_node(self):
        self.cap.release()
        self.esp1.close()
        self.esp2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PiCameraMotor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
