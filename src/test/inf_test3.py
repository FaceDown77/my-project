from flask import Flask, Response
import torch
import cv2
import RPi.GPIO as GPIO
from time import sleep, strftime, time
from threading import Thread, Timer
import threading
import os
import serial
import socket

app = Flask(__name__)

ser = serial.Serial('/dev/serial0', 9600, timeout=1)

def send_manual_mode_signal():
    try:
        ser.write(b'manual\n')
        print("Manual mode signal sent via serial")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

green_led_pin = 17  # 초록색 LED 핀
red_led_pin = 18    # 빨간색 LED 핀
buzzer_pin = 19     # 부저 핀

# GPIO 핀 설정
GPIO.setup(green_led_pin, GPIO.OUT)
GPIO.setup(red_led_pin, GPIO.OUT)
GPIO.setup(buzzer_pin, GPIO.OUT)

# YOLOv5 모델 불러오기 (CPU 사용)
model1 = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ele95/yolov5/best.pt', _verbose=False)
model2 = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/ele95/yolov5/yolov5s.pt', _verbose=False)

model1.names = ['Person', '-', 'thesis-gun-knife - v4 Yolov8s']
model2.names = ['Knife', 'Pistol']

# 카메라에서 입력 받기
cap = cv2.VideoCapture(0)

# 해상도 낮추기 (처리 속도 향상을 위해)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

frame_skip = 5
frame_count = 0

prev_person_box = None
prev_time = time()
movement_threshold = 100 

capturing = False
capture_timer = None

save_dir = '/home/ele95/Pictures'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

def setup_connection(server_ip, server_port=5001):
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((server_ip, server_port))  # 서버에 한번 연결
        print(f"Connected to {server_ip} on port {server_port}")
        return client_socket
    except Exception as e:
        print(f"Failed to connect to server: {e}")
        return None

# 파일 전송 (이미 연결된 소켓을 사용)
# 파일 전송 (이미 연결된 소켓을 사용)
def send_file(client_socket, file_path):
    try:
        file_name = file_path.split('/')[-1]  # 파일 이름 추출
        client_socket.send((file_name + '\n').encode('utf-8'))  # 파일 이름 전송
        print(f"Sending file: {file_name}")

        # 파일 내용 전송
        with open(file_path, 'rb') as f:
            while True:
                file_data = f.read(1024)
                if not file_data:
                    break
                client_socket.send(file_data)

        # 파일 전송 후 끝을 알리는 <END> 전송
        client_socket.send(b'<END>')
        print(f"{file_name} successfully sent with end marker.")
    except Exception as e:
        print(f"Error sending file: {e}")


# 2초마다 사진을 저장하고 전송하는 함수
def save_image_and_send(client_socket, frame, label):
    timestamp = strftime("%Y%m%d_%H%M%S")
    image_path = f"{save_dir}/{label}_{timestamp}.jpg"
    cv2.imwrite(image_path, frame)
    print(f"Captured {label}_{timestamp}.jpg")

    # 파일 전송
    send_file(client_socket, image_path)

# LED와 부저를 깜빡이기 위한 함수
def blink_led_buzzer(led_pin, buzzer_pin=None, blink_time=1):
    GPIO.output(led_pin, GPIO.HIGH)
    if buzzer_pin:
        GPIO.output(buzzer_pin, GPIO.HIGH)
    sleep(blink_time)
    GPIO.output(led_pin, GPIO.LOW)
    if buzzer_pin:
        GPIO.output(buzzer_pin, GPIO.LOW)

# Flask를 통해 MJPEG 스트림을 제공하는 함수
def generate_frames():
    while True:
        success, frame = cap.read()
        if not success:
            break
        else:
            # JPEG로 인코딩
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()

            # MJPEG 스트림 형식으로 반환
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# Flask 서버를 실행하는 스레드
def run_flask():
    app.run(host='0.0.0.0', port=5002)

flask_thread = threading.Thread(target=run_flask)
flask_thread.daemon = True
flask_thread.start()

# 서버와의 연결을 유지한 상태에서 웹캠 피드 처리
client_socket = setup_connection('192.168.0.12', 5001)  # 서버에 미리 연결
if client_socket is None:
    print("Could not establish connection to the server. Exiting...")
    exit(1)

# 실시간 웹캠 피드에서 객체 탐지
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break

    frame_count += 1

    # 매 5 프레임마다 한 번씩 처리
    if frame_count % frame_skip == 0:
        img = cv2.resize(frame, (320, 240))

        # 모델 1에서 탐지 수행 (Person)
        results1 = model1(img, size=320)
        detections1 = results1.pandas().xyxy[0]

        # 모델 2에서 탐지 수행 (Knife, Gun)
        results2 = model2(img, size=320)
        detections2 = results2.pandas().xyxy[0]

        person_detected = False
        weapon_detected = False
        fast_movement_detected = False

        for _, row in detections1.iterrows():
            if row['name'] == 'Person':
                person_detected = True
                xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])

                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
                cv2.putText(frame, 'Person', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

                current_time = time()
                if prev_person_box:
                    prev_xmin, prev_ymin, prev_xmax, prev_ymax = prev_person_box
                    distance = ((xmin - prev_xmin) ** 2 + (ymin - prev_ymin) ** 2) ** 0.5
                    time_diff = current_time - prev_time
                    speed = distance / time_diff

                    if speed > movement_threshold:
                        print(f"Fast movement detected: {speed:.2f} pixels/second")
                        fast_movement_detected = True

                        GPIO.output(red_led_pin, GPIO.HIGH)
                        GPIO.output(buzzer_pin, GPIO.HIGH)

                        if not capturing:
                            capturing = True
                            timestamp = strftime("%Y%m%d_%H%M%S")
                            cv2.imwrite(f"{save_dir}/fast_movement_{timestamp}.jpg", frame)
                            print(f"Captured fast_movement_{timestamp}.jpg")

                            capture_timer = Timer(2, save_image_and_send, args=(client_socket, frame, 'fast_movement'))
                            capture_timer.start()

                prev_person_box = (xmin, ymin, xmax, ymax)
                prev_time = current_time

        for _, row in detections2.iterrows():
            if row['name'] == 'Knife':
                weapon_detected = True
                xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                cv2.putText(frame, 'Knife', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                send_manual_mode_signal()

                if not capturing:
                    capturing = True
                    timestamp = strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"{save_dir}/detected_knife_{timestamp}.jpg", frame)
                    print(f"Captured detected_knife_{timestamp}.jpg")

                    threading.Thread(target=save_image_and_send, args=(client_socket, frame, 'detected_knife')).start()

        if person_detected and not fast_movement_detected:
            threading.Thread(target=blink_led_buzzer, args=(green_led_pin,)).start()

        if fast_movement_detected or weapon_detected:
            threading.Thread(target=blink_led_buzzer, args=(red_led_pin, buzzer_pin)).start()

    cv2.imshow('YOLO Detection - Person and Weapon', frame)

    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

# 프로그램 종료 시 GPIO 핀 해제
GPIO.output(green_led_pin, GPIO.LOW)
GPIO.output(red_led_pin, GPIO.LOW)
GPIO.output(buzzer_pin, GPIO.LOW)
GPIO.cleanup()

# 시리얼 포트 닫기
ser.close()

# 타이머 해제
if capture_timer:
    capture_timer.cancel()

# 서버와의 연결 해제
if client_socket:
    client_socket.close()
