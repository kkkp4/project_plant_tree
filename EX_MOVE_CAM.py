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
        # ESP32 #1
        self.ser1 = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        # ESP32 #2
        self.ser2 = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        self.speed = 100
        self.get_logger().info(
            "Teleop ready with 2 ESP32!\n"
            "Controls:\n"
            "  w = Forward\n"
            "  s = Backward\n"
            "  a = Left\n"
            "  d = Right\n"
            "  space = STOP ALL (servo + stepper + dc motor)\n"
            "  + / - = Speed Up / Down\n"
            "  j = Stepper1 Forward 1 rev\n"
            "  k = Stepper1 Backward 1 rev\n"
            "  l = Stepper2 +45° (incremental)\n"
            "  o = Servo1 Toggle (0° ↔ 150°)\n"
            "  u = Servo2 Toggle (0° ↔ 90°)\n"
            "  i = DC Motor1 Toggle (ON/OFF)\n"
            "  q = Quit\n"
            "Note: คำสั่งทั้งหมดส่งไป ESP32 ทั้งสองตัว"
        )

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

            # Movement (WASD)
            if key == 'w':
                self.send_cmd('F')
            elif key == 's':
                self.send_cmd('B')
            elif key == 'a':
                self.send_cmd('L')
            elif key == 'd':
                self.send_cmd('R')

            # STOP ALL
            elif key == ' ':
                self.send_cmd('STOP_ALL')

            # Speed control
            elif key == '+':
                self.speed = min(255, self.speed + 10)
                self.get_logger().info(f"Speed increased: {self.speed}")
            elif key == '-':
                self.speed = max(0, self.speed - 10)
                self.get_logger().info(f"Speed decreased: {self.speed}")

            # Stepper 1
            elif key == 'j':
                self.send_cmd('STEP1_FWD')
            elif key == 'k':
                self.send_cmd('STEP1_BWD')

            # Stepper 2
            elif key == 'l':
                self.send_cmd('STEP2_45')

            # Servo1
            elif key == 'o':
                self.send_cmd('SERVO1_TOGGLE')

            # Servo2
            elif key == 'u':
                self.send_cmd('SERVO2_TOGGLE')

            # DC Motor1
            elif key == 'i':
                self.send_cmd('MOTOR1_TOGGLE')

            # Quit
            elif key == 'q':
                self.get_logger().info("Exiting teleop...")
                break

    def send_cmd(self, direction, target="all"):
        cmd = f"{direction}:{self.speed}\n"

        try:
            if target == "esp1":
                self.ser1.write(cmd.encode('utf-8'))
                self.get_logger().info(f"Sent {cmd.strip()} to ESP32 #1")
            elif target == "esp2":
                self.ser2.write(cmd.encode('utf-8'))
                self.get_logger().info(f"Sent {cmd.strip()} to ESP32 #2")
            else:  # ส่งไปทั้งสองตัว
                self.ser1.write(cmd.encode('utf-8'))
                self.ser2.write(cmd.encode('utf-8'))
                self.get_logger().info(f"Sent {cmd.strip()} to BOTH ESP32s")
        except Exception as e:
            self.get_logger().error(f"Error sending command: {e}")


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
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
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
    # Start Flask in background thread
    flask_thread = threading.Thread(target=run_flask, daemon=True)
    flask_thread.start()

    # Run ROS2 node
    rclpy.init(args=args)
    node = TeleopSerial()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
