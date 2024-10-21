import torch
import cv2
from time import sleep, strftime, time
from threading import Thread, Timer
import threading
import os
import serial

ser = serial.Serial('/dev/serial0', 9600, timeout=1)

def send_manual_mode_signal():
    try:
        # 칼이 탐지되면 'manual' 신호 전송
        ser.write(b'manual\n')
        print("Manual mode signal sent via serial")
    except serial.SerialException as e:
        print(f"Serial communication error: {e}")


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

# 프레임 스킵 변수
frame_skip = 5  # 5 프레임마다 한 번씩 처리
frame_count = 0

# 빠른 움직임 감지에 필요한 변수들
prev_person_box = None  # 이전에 감지된 사람의 바운딩 박스 좌표
prev_time = time()      # 이전에 감지된 시간
movement_threshold = 100  # 초당 100픽셀 이상 이동 시 빠른 움직임으로 간주

# 사진 저장 주기 타이머 상태
capturing = False
capture_timer = None

# 이미지 저장 경로 설정 (라즈베리파이 로컬 경로)
save_dir = '/home/ele95/Pictures'
if not os.path.exists(save_dir):
    os.makedirs(save_dir)

# 2초마다 사진을 저장하는 함수
def save_image_continuously(frame, label):
    global capturing, capture_timer
    timestamp = strftime("%Y%m%d_%H%M%S")
    cv2.imwrite(f"{save_dir}/{label}_{timestamp}.jpg", frame)
    print(f"Captured {label}_{timestamp}.jpg")

    # 일정 시간이 지나면 사진 촬영을 종료
    capturing = False  # 다시 사진 촬영이 가능하게 함

# LED와 부저를 깜빡이기 위한 함수
def blink_led_buzzer(led, buzzer=None, blink_time=1):
    led.on()
    if buzzer:
        buzzer.on()
    sleep(blink_time)
    led.off()
    if buzzer:
        buzzer.off()

# 실시간 웹캠 피드에서 객체 탐지
while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # 매 5 프레임마다 한 번씩 처리
    if frame_count % frame_skip == 0:
        # YOLO 모델에 맞게 프레임 크기를 320x240으로 조정
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

        # 사람 객체 탐지 결과를 바운딩 박스로 표시
        for _, row in detections1.iterrows():
            if row['name'] == 'Person':
                person_detected = True
                xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])

                # 사람 위치를 박스로 그리기
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (255, 0, 0), 2)
                cv2.putText(frame, 'Person', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 0), 2)

                # 이전에 사람의 위치가 있었을 때, 현재 위치와 비교해 빠른 움직임 감지
                current_time = time()
                if prev_person_box:
                    prev_xmin, prev_ymin, prev_xmax, prev_ymax = prev_person_box
                    distance = ((xmin - prev_xmin) ** 2 + (ymin - prev_ymin) ** 2) ** 0.5  # 이동 거리 계산
                    time_diff = current_time - prev_time  # 시간 차이 계산
                    speed = distance / time_diff  # 초당 픽셀 이동 속도

                    if speed > movement_threshold:
                        print(f"Fast movement detected: {speed:.2f} pixels/second")
                        fast_movement_detected = True
                        # 빠른 움직임 시 사진 촬영
                        if not capturing:
                            capturing = True
                            timestamp = strftime("%Y%m%d_%H%M%S")
                            cv2.imwrite(f"{save_dir}/fast_movement_{timestamp}.jpg", frame)
                            print(f"Captured fast_movement_{timestamp}.jpg")

                            # 2초 후에 다시 사진 촬영
                            capture_timer = Timer(2, save_image_continuously, args=(frame, 'fast_movement'))
                            capture_timer.start()

                # 현재 위치와 시간을 이전 값으로 저장
                prev_person_box = (xmin, ymin, xmax, ymax)
                prev_time = current_time

        # 무기 객체 탐지 결과를 바운딩 박스로 표시
        for _, row in detections2.iterrows():
            if row['name'] == 'Knife':
                weapon_detected = True
                xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
                cv2.putText(frame, 'Knife', (xmin, ymin - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                send_manual_mode_signal()

                # 칼이 탐지되었을 때 사진 촬영
                if not capturing:
                    capturing = True
                    timestamp = strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"{save_dir}/detected_knife_{timestamp}.jpg", frame)
                    print(f"Captured detected_knife_{timestamp}.jpg")

                    # 2초 후에 다시 사진 촬영
                    capture_timer = Timer(2, save_image_continuously, args=(frame, 'detected_knife'))
                    capture_timer.start()

        # 사람 탐지 시 초록색 LED 깜빡임 (스레드에서 실행)
        if person_detected and not fast_movement_detected:
            threading.Thread(target=blink_led_buzzer, args=(green_led,)).start()

        # 빠른 움직임 또는 무기 탐지 시 빨간색 LED와 부저 깜빡임 (스레드에서 실행)
        if fast_movement_detected or weapon_detected:
            threading.Thread(target=blink_led_buzzer, args=(red_led, buzzer)).start()

    # 결과 프레임 출력
    cv2.imshow('YOLO Detection - Person and Weapon', frame)

    # 'q' 키를 누르면 종료
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 자원 해제
cap.release()
cv2.destroyAllWindows()

# 프로그램 종료 시 LED 및 부저 끄기
ser.close()

# 타이머 해제
if capture_timer:
    capture_timer.cancel()
