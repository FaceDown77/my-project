import threading
import RPi.GPIO as GPIO
import time
import serial

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

IN1, IN2, ENA = 23, 24, 17  # 우측
IN3, IN4, ENB = 27, 22, 18  # 좌측

SENSORS = [{"TRIG": 2, "ECHO": 3}, {"TRIG": 10, "ECHO": 9}, {"TRIG": 20, "ECHO": 21}, {"TRIG": 5, "ECHO": 6}]

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

for sensor in SENSORS:
    GPIO.setup(sensor["TRIG"], GPIO.OUT)
    GPIO.setup(sensor["ECHO"], GPIO.IN)

# PWM 설정
pwm_left = GPIO.PWM(ENB, 1000)
pwm_right = GPIO.PWM(ENA, 1000)

pwm_left.start(0)
pwm_right.start(0)

sensor_distances = [0, 0, 0, 0]
distance_lock = threading.Lock()

# 시리얼 포트 설정 (라즈베리파이의 기본 시리얼 포트)
ser = serial.Serial('/dev/serial0', 9600, timeout=1)

# 모드 설정 (초기 모드는 자동 모드)
manual_mode = False

def left_forward():
    GPIO.output(IN3, GPIO.HIGH)
    GPIO.output(IN4, GPIO.LOW)

def left_back():
    GPIO.output(IN3, GPIO.LOW)
    GPIO.output(IN4, GPIO.HIGH)

def right_forward():
    GPIO.output(IN1, GPIO.LOW)
    GPIO.output(IN2, GPIO.HIGH)

def right_back():
    GPIO.output(IN1, GPIO.HIGH)
    GPIO.output(IN2, GPIO.LOW)

def measure_distance(sensor_index):
    TRIG = SENSORS[sensor_index]["TRIG"]
    ECHO = SENSORS[sensor_index]["ECHO"]

    while True:
        GPIO.output(TRIG, False)
        time.sleep(0.00002)
        GPIO.output(TRIG, True)
        time.sleep(0.00001)
        GPIO.output(TRIG, False)

        start = time.time()
        while GPIO.input(ECHO) == 0:
            start = time.time()

        while GPIO.input(ECHO) == 1:
            stop = time.time()

        check_time = stop - start
        distance = check_time * 34300 / 2

        if 2 < distance < 400:
            with distance_lock:
                sensor_distances[sensor_index] = distance

        time.sleep(0.1)

def straight_motor(speed):
    left_forward()
    right_forward()
    pwm_left.ChangeDutyCycle(abs(speed))
    pwm_right.ChangeDutyCycle(abs(speed))

def straight_right(speed):
    left_forward()
    right_forward()
    pwm_left.ChangeDutyCycle(abs(speed))
    pwm_right.ChangeDutyCycle(abs(0))

def straight_left(speed):
    left_forward()
    right_forward()
    pwm_left.ChangeDutyCycle(abs(0))
    pwm_right.ChangeDutyCycle(abs(speed))

def back_motor(speed):
    left_back()
    right_back()
    pwm_left.ChangeDutyCycle(abs(speed))
    pwm_right.ChangeDutyCycle(abs(speed))

def rotate_left(speed):
    left_back()
    right_forward()
    pwm_left.ChangeDutyCycle(abs(speed))
    pwm_right.ChangeDutyCycle(abs(speed))

def rotate_right(speed):
    left_forward()
    right_back()
    pwm_left.ChangeDutyCycle(abs(speed))
    pwm_right.ChangeDutyCycle(abs(speed))

def stop_motors():
    pwm_left.ChangeDutyCycle(0)
    pwm_right.ChangeDutyCycle(0)

def switch_to_manual_mode():
    global manual_mode
    manual_mode = True
    stop_motors()
    print("Manual mode activated")

try:
    threads = []
    for i in range(4):
        thread = threading.Thread(target=measure_distance, args=(i,))
        thread.daemon = True
        thread.start()
        threads.append(thread)

    while True:
        if manual_mode:
            # 수동 모드일 때는 모터 제어 중지 (필요시 수동 제어 로직 추가 가능)
            continue

        with distance_lock:
            Rcenter_distance = sensor_distances[0]
            Lcenter_distance = sensor_distances[1]
            right_distance = sensor_distances[2]
            left_distance = sensor_distances[3]

        # 특정 조건에서 수동 모드로 전환
        if Rcenter_distance < 30 or Lcenter_distance < 30:
            if left_distance > right_distance:
                rotate_left(30)
            else:
                rotate_right(30)
        elif left_distance < 20:
            straight_right(20)
        elif right_distance < 20:
            straight_left(20)
        else:
            straight_motor(20)

        # 시리얼로부터 데이터 수신
        data = ser.readline().decode().strip()
        if data == "manual":
            switch_to_manual_mode()

        time.sleep(0.1)

finally:
    pwm_left.stop()
    pwm_right.stop()
    GPIO.cleanup()
    ser.close()
