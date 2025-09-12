import rclpy
from rclpy.node import Node
import serial
import sys, tty, termios

class TeleopSerial(Node):
    def __init__(self):
        super().__init__('teleop_serial')
        self.ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
        self.speed = 100
        self.get_logger().info(
            "Teleop ready!\n"
            "Controls:\n"
            "  w = Forward, s = Backward, a = Left, d = Right\n"
            "  Space = Stop, + = Speed Up, - = Speed Down\n"
            "  j = Stepper1 Forward 1 rev\n " 
            "  k = Stepper1 Backward 1 rev\n"
            "  l = Stepper2 +45° (incremental)\n"
            "  o = Servo Toggle (0° ↔ 150°)\n"
            "  q = Quit"
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
            # move
            if key == 'w':
                self.send_cmd('F')
            elif key == 's':
                self.send_cmd('B')
            elif key == 'a':
                self.send_cmd('L')
            elif key == 'd':
                self.send_cmd('R')
            elif key == ' ':
                self.send_cmd('S')
            # speed
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
            # Servo
            elif key == 'o':
                self.send_cmd('SERVO_TOGGLE')
            elif key == 'q':
                self.get_logger().info("Exiting teleop...")
                break

    def send_cmd(self, direction):
        cmd = f"{direction}:{self.speed}\n"
        self.ser.write(cmd.encode('utf-8'))
        self.get_logger().info(f"Sent {cmd.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopSerial()
    node.loop()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
