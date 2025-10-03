import rclpy
from rclpy.node import Node
import serial
import sys, tty, termios
import threading
from flask import Flask, Response
import cv2

# -------------------- ROS2 TELEOP NODE --------------------
class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        # เชื่อมต่อ ESP32 สองตัว (ปรับพอร์ตตามจริง)
        self.ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        self.speed = 100  # ค่า default

        self.get_logger().info(
            "\nTeleop ready!\n"
            "Controls:\n"
            "  w/s/a/d = Movement\n"
            "  space   = STOP ALL\n"
            "  +/-     = Speed Up / Down\n"
            "  j/k     = Stepper1 FWD / BWD\n"
            "  l       = Stepper2 +45°\n"
            "  o       = Servo1 Toggle (70° ↔ 150°)\n"
            "  u       = Servo2 Toggle (0° ↔ 90°)\n"
            "  z/x     = Servo1 direct 70° / 150°\n"
            "  c/v     = Servo2 direct 0° / 90°\n"
            "  i       = DC Motor1 Toggle\n"
            "  q       = Quit"
        )

    def getch(self):
        """ อ่าน key จาก keyboard """
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

    def loop(self):
        while rclpy.ok():
            key = self.getch().lower()

            # Movement
            if key == 'w': self.send_cmd('F')
            elif key == 's': self.send_cmd('B')
            elif key == 'a': self.send_cmd('L')
            elif key == 'd': self.send_cmd('R')

            # STOP ALL
            elif key == ' ': self.send_cmd('STOP_ALL')

            # Speed
            elif key == '+':
                self.speed = min(255, self.speed + 10)
                self.get_logger().info(f"Speed increased: {self.speed}")
            elif key == '-':
                self.speed = max(0, self.speed - 10)
                self.get_logger().info(f"Speed decreased: {self.speed}")

            # Stepper
            elif key == 'j': self.send_cmd('STEP1_FWD')
            elif key == 'k': self.send_cmd('STEP1_BWD')
            elif key == 'l': self.send_cmd('STEP2_45')

            # Servo Toggle
            elif key == 'o': self.send_cmd('SERVO1_TOGGLE')
            elif key == 'u': self.send_cmd('SERVO2_TOGGLE')

            # Servo Direct Angle
            elif key == 'z': self.send_cmd('SERVO1', value=70)
            elif key == 'x': self.send_cmd('SERVO1', value=150)
            elif key == 'c': self.send_cmd('SERVO2', value=0)
            elif key == 'v': self.send_cmd('SERVO2', value=90)

            # DC Motor
            elif key == 'i': self.send_cmd('MOTOR1_TOGGLE')

            # Quit
            elif key == 'q':
                self.get_logger().info("Exiting teleop...")
                break

    def send_cmd(self, action, value=None, target="all"):
        """ ส่งคำสั่งไป ESP32 """
        if value is not None:
            cmd = f"{action}:{value}\n"
        else:
            cmd = f"{action}:{self.speed}\n"

        if target == "esp1":
            self.ser1.write(cmd.encode())
            self.get_logger().info(f"Sent {cmd.strip()} -> ESP32 #1")
        elif target == "esp2":
            self.ser2.write(cmd.encode())
            self.get_logger().info(f"Sent {cmd.strip()} -> ESP32 #2")
        else:
            self.ser1.write(cmd.encode())
            self.ser2.write(cmd.encode())
            self.get_logger().info(f"Sent {cmd.strip()} -> BOTH ESP32s")


# -------------------- FLASK VIDEO STREAM --------------------
app = Flask(__name__)
camera = cv2.VideoCapture(0)
camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
camera.set(cv2.CAP_PROP_FPS, 30)

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host="0.0.0.0", port=5000, threaded=True)


# -------------------- MAIN --------------------
def main(args=None):
    # Flask run in background
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Run ROS2 Node
    rclpy.init(args=args)
    node = TeleopSerial()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
