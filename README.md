# 3조 : 자율주행 패트롤카 🚓 (Guard Rover)
  - 자율주행 로봇과 객체인식이 적용된 순찰 로봇 <br>
![C++](https://img.shields.io/badge/C++-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)  ![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)  ![Linux](https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black) <br> ![Raspberry Pi](https://img.shields.io/badge/Raspberry_Pi-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white)  ![Arduino](https://img.shields.io/badge/Arduino-00979D?style=for-the-badge&logo=arduino&logoColor=white)  ![YOLO](https://img.shields.io/badge/YOLO-Ultralytics-00FFFF?style=for-the-badge&logo=data:image/svg+xml;base64,PHN2ZyB4bWxucz0iaHR0cDovL3d3dy53My5vcmcvMjAwMC9zdmciIHdpZHRoPSIyMDAiIGhlaWdodD0iMjAwIiB2aWV3Qm94PSIwIDAgMzAgMzAiPjxjaXJjbGUgY3g9IjE1IiBjeT0iMTUiIHI9IjE1IiBmaWxsPSIjMDBGRkZGIiAvPjx0ZXh0IHg9IjE1IiB5PSIxNiIgdGV4dC1hbmNob3I9Im1pZGRsZSIgZmlsbD0iI0ZGRiIgZm9udC1zaXplPSIxMCIgZm9udC1mYW1pbHk9IkFyaWFsLEhlbHZldGljYSI+WW9sbzwvdGV4dD48L3N2Zz4=)
  ![Flask](https://img.shields.io/badge/Flask-000000?style=for-the-badge&logo=flask&logoColor=white) <br>
![결과물 사진](doc/images/RC-Car_image.jpg)
<br><br>

---

## 📑프로젝트 간략 소개
- ~~자율주행을 통해~~ 구역을 순찰하며 카메라로 침입자를 감지하는 로봇
  - ~~ROS와 2D Lidar 센서를 기반으로 로봇이 이동~~
  - (수정) 장착된 초음파센서를 통해 장애물 감지 및 회피하고 구역을 순찰
  - 장착된 카메라를 통해 순찰 중 발견한 사람을 감지하는 역할을 수행

## 🖥️프로젝트 내용

### 🎯목적
- 사람이 커버하기 힘든 공간을 사람을 대신하여 주어진 영역을 순찰하는 로봇

### 🕵️문제
- 인구가 줄어감에 따라 사람이 커버하기 힘든 넓은 영역에 대해서 순찰 및 감시를 보조할 수단이 필요함

### 🎳목표
- a. ~~라이다센서를 통해 주변을 인식하여 자율주행 가능한 로봇을 구현.~~
- (수정) a-1. 초음파센서를 기반으로 전면의 장애물을 회피하며 공간을 주행하는 로봇 구현.
- b. 로봇이 카메라를 통해 주행 중 영상을 획득.
- c. 획득한 영상에서 무기를 소지한 사람을 인식하고 사용자에게 알림을 보낸다.

### 📅 일정
```mermaid
gantt
    title 작업 일정
    dateFormat YYYY-MM-DD
    section 계획
        아이디어 회의    :2024-09-23, 2d
        초기 일정계획 작성    :2024-09-24, 2d
        시스템 및 구조 작성    :2024-09-23, 3d
    section 개발
        위해행위 포착 모델 작성    :2024-09-26, 2d
        무기 및 흉기 인식 모델 작성    :2024-09-26, 2d
        필수 필요부품 구매리스트 작성 및 구매    :2024-09-30, 3d
        ROS2&라이다 적용 검토    :2024-10-07, 9d
        객체인식 모델 트레이닝-1    :2024-10-07, 2d
        모터제어 로직 작성    :2024-10-07, 5d
        객체인식 모델 트레이닝-2    :2024-10-10, 2d
    section 테스트
        ROS2 라이다센서 테스트    :2024-10-10, 6d
        ROS2&라이다 DROP 및 계획 수정    :2024-10-15, 1d
        RC카 전원공급 및 간단동작 테스트    :2024-10-11, 7d
        객체인식 테스트 및 개선    :2024-10-14, 6d
        RC카 파트 시험주행    :2024-10-17, 3d
        RC카 동작 플로우 개선    :2024-10-17, 3d
    section 발표준비
        발표자료 작성    :2024-10-21, 2d
        D-day    :2024-10-23, 1d
```

---

### 🧑‍🤝‍🧑팀 구성원
- Members
  | Name | Role |
  |----|----|
  | 권시우 | RC카 조립, 모터 구동부 작업, 객체인식 모델 트레이닝, 작업파일 병합 |
  | 박정우 | RC카 조립, 무선 컨트롤러 작업, 무선 통신 구성 및 고도화 |
  | 조명근 | 트레이닝 모델 파이 적용 및 최적화 작업, 영상처리 최적화 |
  | 박인혁 | ~~ROS2 베이스의 Lidar Sensor & 자율주행 담당~~ <br> (조장) 시스템 설계, ROS검토, 작업파일 병합 및 디버깅, 프로젝트 관리 |


### ⚙️하드웨어 및 시스템 (작성중, 이미지파일 적용예정)
- 시스템 구성

![시스템 구성도](./doc/images/system_diagram.jpg)
<br><br>

- 입출력 구성

![입출력 구성도](./doc/images/inout_diagram.jpg)
<br><br>

- 동작 플로우

![동작 플로우차트](./doc/images/flowchart_diagram.jpg)
<br><br>

---

## 🎬시연 영상
### 수동주행
![Remote Control_1](doc/images/RC_Manual_Control_1.gif)  ![Remote Control_2](doc/images/RC_Manual_Control_2.gif)
<br>

### 자율주행
![Auto_1](doc/images/RC_Moving_1.gif)  ![Auto_4](doc/images/RC_Moving_4.gif) <br>
![Auto_2](doc/images/RC_Moving_2.gif)  ![Auto_3](doc/images/RC_Moving_3.gif)
<br>

### 객체인식 (사람 및 무기)
![Detection_1](doc/images/SUS_detection_1.gif)  ![Detection_2](doc/images/SUS_detection_2.gif)
### 원격PC 모니터링 현황
![Detection_3](doc/images/Remote_PC_1.gif)  ![Detection_3](doc/images/detection_1.jpg)
<br>

---

## 프리젠테이션 자료
[발표 자료 - PPT](doc/GuardRover_presentation.pptx) <br>
[발표 자료 - PDF](doc/GuardRover_presentation.pdf) <br>
<br>

[🔝맨 위로🔝](#top)
