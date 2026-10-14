# Webots TurtleBot3 사진 촬영 BT

## 파일 위치

이 폴더는 /home/gayeon/py_bt_ros/scenarios/photo_mission 에 둡니다.

- bt_nodes.py: 목표 수신, 출발 위치 저장, 이동, 사진 저장 요청, 복귀
- default_bt.xml: Post-condition / Pre-condition / Action 구조의 행동 트리
- config.yaml: Webots용 토픽, 액션, 서비스, 좌표계, 시뮬레이션 시간 설정
- __init__.py: 시나리오 패키지
- image_capture_service.py: 앞 단계에서 설치한 사진 저장 서비스 (이 압축에는 포함되지 않음)

## 실행

터미널 1: Webots + Nav2

```bash
source /opt/ros/humble/setup.bash
cd ~/webots_ros2_ws
source install/local_setup.bash
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=true
```

터미널 2: 사진 저장 서비스

```bash
source /opt/ros/humble/setup.bash
source ~/webots_ros2_ws/install/local_setup.bash
cd ~/py_bt_ros
python3 scenarios/photo_mission/image_capture_service.py --ros-args -p use_sim_time:=true -p save_dir:=/home/gayeon/robot_photos
```

터미널 3: Behaviour Tree

```bash
source /opt/ros/humble/setup.bash
source ~/webots_ros2_ws/install/local_setup.bash
cd ~/py_bt_ros
python3 main.py --config=scenarios/photo_mission/config.yaml
```

RViz에서 초기 위치가 아직 설정되지 않았다면 2D Pose Estimate로 로봇의 실제 위치와 방향을 지정합니다.
Fixed Frame은 map, 새 SetGoal 도구의 Topic은 /bt/goal_pose로 설정합니다.
BT 시작 로그를 확인한 뒤 이 도구로 목표 위치와 방향을 지정합니다.

## 동작 순서

1. RViz 목표 메시지를 받고 작업 시작
2. map -> base_link TF로 현재 위치와 방향 저장
3. /navigate_to_pose Action으로 목표 이동
4. 도착 성공 시 /capture_image Service 호출
5. 서비스 성공과 이미지 파일 존재 확인
6. 저장한 출발 위치와 방향으로 /navigate_to_pose Action 호출
7. 복귀 성공 시 완료하고 다음 목표 대기

완료 여부는 현재 작업별로 저장합니다. BT tick마다 이동이나 촬영을 다시 요청하지 않습니다.
진행 중 새 목표는 받지 않으며 복귀 완료 후 새 목표를 보냅니다.
이동 실패나 촬영 실패는 완료로 처리하지 않고 실패 로그를 출력한 뒤 다음 목표를 기다립니다.

## 확인할 로그

- Waiting for an RViz goal on /bt/goal_pose
- Mission 1: goal received
- Initial pose saved
- MoveToGoal: goal sent
- MoveToGoal: succeeded
- CaptureImage: requesting photo storage
- CaptureImage: saved /home/gayeon/robot_photos/...
- Return: goal sent
- Return: succeeded
- Mission 1 complete: returned to the initial pose

사진 폴더는 /home/gayeon/robot_photos 입니다.

## 설정

- navigation_timeout: 이동/복귀 각각의 대기 제한(초), 기본 180
- capture_timeout: BT에서 촬영 서비스 응답을 기다리는 제한(초), 기본 10
- 사진 서버 자체의 새 프레임 대기 제한은 기본 3초
- BT 시각화가 필요하면 bt_runner.bt_visualiser.enabled를 true로 변경
- 초기 위치 TF가 없다면 Nav2 초기 위치 설정을 확인
