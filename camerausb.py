from flask import Flask, Response
import cv2

app = Flask(__name__)
camera = cv2.VideoCapture(0)  # ใช้ /dev/video0

def generate_frames():
    while True:
        success, frame = camera.read()
        if not success:
            break
        else:
            # ตรงนี้คุณสามารถใส่โค้ดวัดขนาดได้ในอนาคต
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; bou>

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5000)
