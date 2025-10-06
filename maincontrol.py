import rclpy
from rclpy.node import Node
import serial
import sys, tty, termios
import threading
from flask import Flask, render_template, Response, jsonify
import cv2

# -------------------- Flask App --------------------
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

# แชร์ข้อมูล encoder ไปเว็บ
encoder_data = {'left_cm': 0.0, 'right_cm': 0.0}
initial_offset = {'left': None, 'right': None}

@app.route('/distance')
def get_distance():
    return jsonify(encoder_data)


# -------------------- ROS2 TELEOP NODE --------------------
class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')

        self.ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
        self.speed = 100

        threading.Thread(target=self.read_encoders, daemon=True).start()
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
                self.send_cmd("STOP_ALL")
                self.get_logger().info("Exiting teleop...")
                break

            actions = {
                'w': 'F', 's': 'B', 'a': 'L', 'd': 'R', ' ': 'S',
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
        elif target == "esp2":
            self.ser2.write(cmd.encode())
        else:
            self.ser1.write(cmd.encode())
            self.ser2.write(cmd.encode())
        self.get_logger().info(f"Sent {cmd.strip()} -> BOTH ESP32s")

    def read_encoders(self):
        """อ่านค่า encoder (ไม่แสดงใน terminal)"""
        while rclpy.ok():
            for ser in [self.ser1, self.ser2]:
                try:
                    if ser.in_waiting:
                        line = ser.readline().decode('utf-8').strip()
                        if line.startswith("L_TICKS"):
                            parts = line.split(',')
                            data = {}
                            for p in parts:
                                k, v = p.split(':')
                                data[k] = float(v)

                            left_cm = data['L_DIST'] / 10.0
                            right_cm = data['R_DIST'] / 10.0

                            if initial_offset['left'] is None:
                                initial_offset['left'] = left_cm
                            if initial_offset['right'] is None:
                                initial_offset['right'] = right_cm

                            encoder_data['left_cm'] = round(left_cm - initial_offset['left'], 1)
                            encoder_data['right_cm'] = round(right_cm - initial_offset['right'], 1)
                except Exception:
                    pass  # ไม่ต้องโชว์ error ใน loop encoder


# -------------------- MAIN --------------------
def run_flask():
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # ปิด log การเข้าถึงเว็บ (GET /distance)
    app.run(host='0.0.0.0', port=5000, debug=False, threaded=True)

def main(args=None):
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    rclpy.init(args=args)
    node = TeleopSerial()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
