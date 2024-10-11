# 3조 Team projects repository

## 팀 구성원
  - 권시우
  - 박인혁(조장)
  - 박정우
  - 조명근


### 제출방법

1. 팀구성 및 프로젝트 세부 논의 후, 각 팀은 프로젝트 진행을 위한 Github repository 생성

2. [doc/project/README.md](./doc/project/README.md)을 각 팀이 생성한 repository의 main README.md로 복사 후 팀 프로젝트에 맞게 수정 활용

3. 과제 제출시 `인텔교육 5기 Github repository`에 `New Issue` 생성. 생성된 Issue에 하기 내용 포함되어야 함.

    * Team name : Project Name
    * Project 소개
    * 팀원 및 팀원 역활
    * Project Github repository
    * Project 발표자료 업로드

4. 강사가 생성한 `Milestone`에 생성된 Issue에 추가 

### 평가방법

* [assessment-criteria.pdf](./doc/project/assessment-criteria.pdf) 참고

### 제출현황

### Team: 가드로버(Guard Rover)
<자율주행 패트롤카>
  - 목적: 사람이 커버하기 힘든 공간을 대신하여 주어진 영역을 순찰하는 로봇
  - 문제: 인구가 줄어감에 따라 사람이 커버하기 힘든 넓은 영역에 대해서 이를 보조할 수단이 필요함
  - 목표 
    - 1) 라이다를 통해 주변을 인식하여 자율주행 가능한 로봇을 구현
    - 2) 로봇이 주행중 영상을 획득하고, 무기를 소지한 사람을 인식하여 사용자에게 알림을 보낸다.
  - 방법: Lidar Sensor 활용, RC 로봇 자율주행, 딥러닝 모델 활용
  - HW: 라즈베리파이4 * 2대, 2D Lidar 센서, 초음파 센서, 파이카메라3
* Members
  | Name | Role |
  |----|----|
  | 박인혁 | ROS2 베이스의 Lidar Sensor & 자율주행 담당 / (조장) |
  | 권시우 | RC 차량파트, 흉기 및 사람인식 딥러닝 모델 구현 |
  | 박정우 | RC 차량파트, 흉기 및 사람인식 딥러닝 모델 구현 |
  | 조명근 | 영상처리 모델 라즈베리파이 적용 및 최적화, RC 차량파트 |
* Project Github : https://github.com/FaceDown77/intel-class-PJT03.git
* 발표자료 : https://github.com/FaceDown77/intel-class-PJT03.git/doc/presentation.pptx
