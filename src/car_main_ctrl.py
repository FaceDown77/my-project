import socket
import threading
import time
import serial

import RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# 핀 설정
IN1, IN2, ENA = 23, 24, 17  # 오른쪽 바퀴
IN3, IN4, ENB = 27, 22, 18  # 왼쪽 바퀴
SENSORS = [{"TRIG": 2, "ECHO": 3}, {"TRIG": 10, "ECHO": 9}, {"TRIG": 20, "ECHO": 21}, {"TRIG": 5, "ECHO": 6}]

GPIO.setup([IN1, IN2, ENA, IN3, IN4, ENB], GPIO.OUT)

for sensor in SENSORS:
    GPIO.setup(sensor["TRIG"], GPIO.OUT)
    GPIO.setup(sensor["ECHO"], GPIO.IN)

pwm_left = GPIO.PWM(ENB, 1000)
pwm_right = GPIO.PWM(ENA, 1000)
pwm_left.start(0)
pwm_right.start(0)

sensor_distances = [0, 0, 0, 0]
distance_lock = threading.Lock()

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.bind(("0.0.0.0", 5000))
server_socket.listen(1)

ser = serial.Serial('/dev/serial0', 9600, timeout=1)

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


def move_motors(left_speed, right_speed):
    if right_speed >= 0:
        GPIO.output(IN1, GPIO.LOW)
        GPIO.output(IN2, GPIO.HIGH)
    else:
        GPIO.output(IN1, GPIO.HIGH)
        GPIO.output(IN2, GPIO.LOW)

    if left_speed >= 0:
        GPIO.output(IN3, GPIO.HIGH)
        GPIO.output(IN4, GPIO.LOW)
    else:
        GPIO.output(IN3, GPIO.LOW)
        GPIO.output(IN4, GPIO.HIGH)

    pwm_left.ChangeDutyCycle(abs(left_speed))
    pwm_right.ChangeDutyCycle(abs(right_speed))

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

def automatic_mode():
    with distance_lock:
        Rcenter_distance = sensor_distances[0]
        Lcenter_distance = sensor_distances[1]
        right_distance = sensor_distances[2]
        left_distance = sensor_distances[3]

        if Rcenter_distance < 30 or Lcenter_distance < 30:
            if left_distance > right_distance:
                rotate_left(30)
            else:
                rotate_right(30)
        elif left_distance < 20 and right_distance < 20:
            back_motor(30) 
        elif left_distance < 15:
            straight_right(25)
        elif right_distance < 15:
            straight_left(25)
        else:
            straight_motor(20)

def manual_mode(client_socket, x_value, y_value):
    try:
        max_speed = 35
        speed = (y_value / 100) * max_speed

        if x_value < 0:
            left_speed = speed
            right_speed = speed * (1 + (x_value / 100))
        elif x_value > 0:
            left_speed = speed * (1 - (x_value / 100))
            right_speed = speed
        else:
            left_speed = speed
            right_speed = speed

        move_motors(left_speed, right_speed)
        return True
    except ValueError:
        print("Error parsing received data")
        return False


try:
    current_mode = "automatic" 
    last_sw_state = 1  

    client_socket, addr = server_socket.accept()
    print(f"Connected by {addr}")

    threads = []
    for i in range(4):
        thread = threading.Thread(target=measure_distance, args=(i,))
        thread.daemon = True
        thread.start()
        threads.append(thread)

    while True:
        # 시리얼 데이터 수신
        serial_data = ser.readline().decode().strip()
        if serial_data == "manual" and current_mode == "automatic":
            current_mode = "manual"
            print("Mode switched to manual via serial command")

        # 소켓 데이터 수신
        data = client_socket.recv(1024)
        if not data:
            break
        message = data.decode().strip()

        try:
            if "X:" in message and "Y:" in message and "SW:" in message:
                parts = message.split()
                x_value = int(parts[1].replace(",", ""))
                y_value = int(parts[3])
                sw_value = int(parts[5])  

                if sw_value == 0 and last_sw_state == 1:
                    current_mode = (
                        "manual" if current_mode == "automatic" else "automatic"
                    )
                    print(f"Mode switched to {current_mode} via button press")

                last_sw_state = sw_value  

                if current_mode == "automatic":
                    automatic_mode()
                else:
                    manual_mode(client_socket, x_value, y_value)

        except ValueError:
            print("Error parsing received data")

        time.sleep(0.1)

finally:
    stop_motors()
    GPIO.cleanup()
    if "client_socket" in locals():
        client_socket.close()
    server_socket.close()
    ser.close()
