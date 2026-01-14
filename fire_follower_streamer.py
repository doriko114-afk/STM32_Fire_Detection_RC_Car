from flask import Flask, Response
import cv2
import numpy as np

app = Flask(__name__)

# --- 설정 값 ---
FRAME_WIDTH = 640
FRAME_HEIGHT = 480

# 웹캠 초기화
cap = cv2.VideoCapture(1) # 0번 카메라 사용
cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

def generate_frames():
    # --- 사용자 지정 HSV 색상 범위 ---
    lower_bound = np.array([0, 106, 101])
    upper_bound = np.array([44, 167, 245])

    while True:
        success, frame = cap.read()
        if not success:
            print("오류: 프레임을 읽을 수 없습니다. 스트림 종료.")
            break
        else:
            # --- 불꽃 감지 로직 ---
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_bound, upper_bound)

            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

            largest_contour_area = 0
            target_contour = None

            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_contour_area:
                    largest_contour_area = area
                    target_contour = contour
            
            if target_contour is not None and largest_contour_area > 1000:
                x, y, w, h = cv2.boundingRect(target_contour)
                cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)
                cv2.putText(frame, "FIRE DETECTED!", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()

            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    print("[Python Streamer]: MJPEG 스트리밍 서버를 http://127.0.0.1:8000/video_feed 에서 시작합니다.")
    app.run(host='0.0.0.0', port=8000, threaded=True)
