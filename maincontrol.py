import rclpy
from rclpy.node import Node
import serial
import sys, tty, termios
import threading
from flask import Flask, render_template, Response
import cv2
import os

# -------------------- ROS2 TELEOP NODE --------------------
class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        self.ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        self.speed = 100

        self.get_logger().info("Teleop ready! (Press 'q' to quit)")

    def getch(self):
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
            if key == 'q':
                self.get_logger().info("Exiting teleop...")
                break

            actions = {
                'w': 'F', 's': 'B', 'a': 'L', 'd': 'R', ' ': 'STOP_ALL',
                'j': 'STEP1_FWD', 'k': 'STEP1_BWD', 'l': 'STEP2_45',
                'o': 'SHAKE', 'u': 'MOTOR1_OFF',
                'z': ('SERVO1', 70), 'x': ('SERVO1', 150),
                'c': ('SERVO2', 0), 'v': ('SERVO2', 90),
                'i': 'MOTOR1_ON',
            }

            if key == '+':
                self.speed = min(255, self.speed + 10)
                self.get_logger().info(f"Speed increased: {self.speed}")
            elif key == '-':
                self.speed = max(0, self.speed - 10)
                self.get_logger().info(f"Speed decreased: {self.speed}")
            elif key in actions:
                cmd = actions[key]
                if isinstance(cmd, tuple):
                    self.send_cmd(cmd[0], value=cmd[1])
                else:
                    self.send_cmd(cmd)

    def send_cmd(self, action, value=None, target="all"):
        cmd = f"{action}:{value if value is not None else self.speed}\n"
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


# -------------------- FLASK + OpenCV CAMERA --------------------
app = Flask(__name__, template_folder='templates')

cam0 = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
cam2 = cv2.VideoCapture("/dev/video2", cv2.CAP_V4L2)

for cam in [cam0, cam2]:
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
    cam.set(cv2.CAP_PROP_FPS, 15)

def generate(cam):
    while True:
        success, frame = cam.read()
        if not success:
            break
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        yield (b'--frame\r\nContent-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')

@app.route('/')
def index():
    return render_template('index.html')

@app.route('/cam0')
def cam0_feed():
    return Response(generate(cam0), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/cam2')
def cam2_feed():
    return Response(generate(cam2), mimetype='multipart/x-mixed-replace; boundary=frame')

def run_flask():
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)


# -------------------- MAIN --------------------
def main(args=None):
    # รัน Flask server ใน thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # รัน ROS2 node
    rclpy.init(args=args)
    node = TeleopSerial()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
