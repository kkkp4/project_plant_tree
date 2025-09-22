from flask import Flask, Response
from picamera2 import Picamera2
import cv2

app = Flask(__name__)

# --- Pi Camera v3 (CSI) ---
picam2 = Picamera2()
config = picam2.create_preview_configuration(
    main={"format": "RGB888", "size": (640, 480)}
)
picam2.configure(config)
picam2.start()

# --- USB Webcam (เช่น /dev/video8) ---
usb_cam = cv2.VideoCapture(8)
usb_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
usb_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

def gen_frames():
    while True:
        frame_pi = picam2.capture_array()
        ret, frame_usb = usb_cam.read()

        if ret:
            frame_usb_resized = cv2.resize(frame_usb, (frame_pi.shape[1], frame_pi.shape[0]))
            combined = cv2.hconcat([frame_pi, frame_usb_resized])
        else:
            combined = frame_pi

        ret, buffer = cv2.imencode('.jpg', combined)
        frame = buffer.tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
