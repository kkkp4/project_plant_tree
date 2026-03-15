import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
import time
import json


# ======================
# Load AprilTag data
# ======================
def load_tag():

    try:
        with open("tag_data.json", "r") as f:
            data = json.load(f)

        print("\nLoaded Tag Data:", data)
        return data

    except:
        print("\nNo tag_data.json found")
        return None


class CabbageScanner(Node):

    def __init__(self):

        super().__init__('cabbage_scanner')

        # ======================
        # Load tag
        # ======================
        tag = load_tag()

        if tag is not None:

            self.first_two = tag["first_two"]
            self.middle = tag["middle"]
            self.last_two = tag["last_two"]

        else:

            self.first_two = 0
            self.middle = 0
            self.last_two = 0

        print(f"Tag values -> {self.first_two} | {self.middle} | {self.last_two}")

        # ======================
        # ROS publisher
        # ======================
        self.cmd_pub = self.create_publisher(
            String,
            '/cmd',
            10
        )

        # ======================
        # รับค่าขนาดจาก YOLO
        # ======================
        self.create_subscription(
            Float32,
            '/cabbage_size',
            self.size_callback,
            10
        )

        # ======================
        # รอ FINISH จาก Pi5
        # ======================
        self.create_subscription(
            String,
            '/mission_status',
            self.finish_callback,
            10
        )

        self.latest_size = None
        self.start_scan = False

        # ======================
        # แปลงค่า middle
        # ======================
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

        # ======================
        # Parameters
        # ======================
        self.x1 = self.middle
        self.x2 = self.last_two

        if self.last_two > 0:
            self.n = int(75 / self.last_two)
        else:
            self.n = 0

        print(f"Scan count (n) = {self.n}")

        self.cabbage_sizes = []

    # ======================
    # รับ FINISH จาก Pi
    # ======================
    def finish_callback(self, msg):

        if msg.data == "FINISH" and not self.start_scan:

            print("\nReceived FINISH → Start Scan")

            self.start_scan = True

            time.sleep(2)

            self.scan()

    # ======================
    # รับค่าจาก YOLO
    # ======================
    def size_callback(self, msg):

        self.latest_size = msg.data

    # ======================
    # ส่งคำสั่งหุ่น
    # ======================
    def send_cmd(self, cmd):

        msg = String()
        msg.data = cmd

        print("CMD:", cmd)

        self.cmd_pub.publish(msg)

        time.sleep(2)

    # ======================
    # ตรวจ YOLO ครั้งเดียว
    # ======================
    def detect_once(self):

        return self.latest_size

    # ======================
    # ถ้าไม่เจอ → ขึ้นลง
    # ======================
    def detect_with_retry(self):

        size = self.detect_once()

        if size is not None:
            return size

        for i in range(3):

            print("No cabbage → Move Up/Down")

            self.send_cmd("D:2")
            self.send_cmd("DB:2")

            size = self.detect_once()

            if size is not None:
                return size

        return None

    # ======================
    # Scan cabbage
    # ======================
    def scan(self):

        print("\nMove to first cabbage")

        self.send_cmd(f"D:{self.x1}")

        for i in range(self.n):

            self.send_cmd("S:0")

            size = self.detect_with_retry()

            if size is None:

                self.cabbage_sizes.append(None)

            else:

                self.cabbage_sizes.append(size)

            if i < self.n - 1:

                self.send_cmd(f"D:{self.x2}")

        # ======================
        # ตรวจครบแล้ว
        # ======================
        print("\nFinished scanning")

        self.send_cmd("D:20")
        self.send_cmd("S:0")

        # ======================
        # แสดงผล
        # ======================
        print("\nCabbage Results")

        for i, size in enumerate(self.cabbage_sizes):

            if size is None:
                print(f"Cabbage {i+1} : Not Found")
            else:
                print(f"Cabbage {i+1} : {size:.1f} cm")

        # ======================
        # Save JSON
        # ======================
        self.save_data()

    # ======================
    # Save data
    # ======================
    def save_data(self):

        data = {
            "cabbage_sizes": self.cabbage_sizes
        }

        with open("cabbage_data.json", "w") as f:

            json.dump(data, f, indent=4)

        print("\nSaved to cabbage_data.json")


# ======================
# Main
# ======================
def main(args=None):

    rclpy.init(args=args)

    node = CabbageScanner()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
