# doosan-m0609-ros2 — 두산 M0609 + ROS 2 Humble (MoveIt 사전 검증 · RViz 미러)

로봇 제어(GUI · 그리퍼 · Pick & Place)는 DRFL 저장소 [doosan-m0609-drfl](https://github.com/srbae7115/doosan-m0609-drfl)에서 하고,
이 저장소는 그 옆에서 쓰는 ROS 2 도구입니다(선택). 학생용 순서는 강의 설치 가이드 **8.6 ~ 8.7**과 같습니다.

- **사전 검증**과 **RViz 미러**는 로봇 제어권을 가져가지 않으므로 DRFL GUI와 함께 써도 됩니다.
- 공식 ROS 2 드라이버로 로봇을 직접 움직이는 실물 모드(`dsr_bringup2 ... mode:=real`, 아래 참고 기록)는 제어권을 가져가므로
  DRFL GUI와 동시에 쓰지 않습니다.

## 설치
### 0) Docker (가상 모드 에뮬레이터용)
- **WSL2**: Windows에 Docker Desktop 설치 → Settings → Resources → WSL integration 에서 이 Ubuntu 켜기 (아래 apt 두 줄은 건너뜀)
- **순수 Ubuntu**:
```bash
sudo apt install -y docker.io
sudo usermod -aG docker $USER
```
다시 로그인한 뒤 확인합니다 (WSL2는 PowerShell에서 `wsl --shutdown` 후 다시 열기).
```bash
docker --version
docker run --rm hello-world     # "Hello from Docker!" 가 나오면 준비 완료
```
- `permission denied` → docker 그룹이 아직 적용 안 됨. Ubuntu 데스크톱은 다시 로그인해도 이전 세션이 남아 반영 안 될 때가 있음 →
  `newgrp docker` 실행 후 같은 터미널에서 다시 확인 (그 터미널에만 적용, 한 번 재부팅하면 이후 불필요)
- `Cannot connect to the Docker daemon` → Docker가 꺼져 있음 (WSL2 + docker.io: `sudo service docker start`, Docker Desktop: Windows에서 앱 실행)

### 1) 패키지 · 저장소 · 설치 스크립트
```bash
sudo apt-get install -y libpoco-dev ros-humble-ros2-controllers dbus-x11
git clone https://github.com/srbae7115/doosan-m0609-ros2.git ~/doosan_ros2_ws
~/doosan_ros2_ws/scripts/setup_doosan_robot2.sh
```
`setup_doosan_robot2.sh`: 공식 [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) humble(8653b04)을 `src/`에 클론 →
안전 패치(`patches/`) 적용 → 에뮬레이터 이미지 다운로드 → 필요한 패키지만 빌드 → `tools/flange_pose` 빌드

> **메모리**: `dsr_controller2.cpp` 한 파일 컴파일에 RAM 약 6 GB가 필요합니다. 부족하면
> `c++: fatal error: Killed signal terminated program cc1plus` 로 실패하므로, 스크립트가 RAM+스왑이 7 GB 미만이면 빌드 전에 멈추고 안내합니다.
> - WSL2: `%UserProfile%\.wslconfig` 의 `[wsl2]` 아래에 `memory=8GB`, `swap=8GB` → PowerShell에서 `wsl --shutdown` → 스크립트 다시 실행
> - 순수 Ubuntu: `sudo fallocate -l 8G /swapfile2 && sudo chmod 600 /swapfile2 && sudo mkswap /swapfile2 && sudo swapon /swapfile2` → 다시 실행
>
> 이미 빌드된 패키지는 건너뛰므로 다시 실행해도 처음부터 하지 않습니다. 빌드에 5분 이상 걸릴 수 있습니다.

| 경로 | 내용 |
|---|---|
| `src/m0609_mirror/` | RViz 미러 패키지 (DRFL 읽기 전용 → `/joint_states`, 책상·그리퍼·경로 마커) |
| `scripts/table_points.py` | 직접 교시로 책상 측정 (책상이 바뀐 경우에만) |
| `scripts/pp_scene.yaml`, `pp_scene_check.py` | MoveIt 장면 + 티칭 Pick & Place 사전 검증 (로봇 이동 없음) |
| `scripts/moveit_*.py` | MoveIt 관절 이동 시험 |
| `tools/flange_pose.cpp` | 플랜지 자세 읽기 (읽기 전용) |
| `patches/` | 공식 드라이버 안전 패치 |
| `table_points.json` | 고정 책상값을 만든 원본 측정 데이터 |

## 사용법 (학생용)

**새 터미널마다 맨 먼저 `source` 를 실행해야 합니다.** 빼먹으면 `Package 'dsr_bringup2' not found` /
`Package 'm0609_mirror' not found` 가 납니다.

### 1) Pick & Place 사전 검증 (로봇은 움직이지 않음)
DRFL GUI에서 home / pick / place 를 저장한 뒤 실행합니다.

**터미널 1: 가상 모드 MoveIt** (검증이 끝날 때까지 켜 둡니다)
```bash
source ~/doosan_ros2_ws/install/setup.bash
```
```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1
```
RViz 창이 뜨고 `You can start planning now!` 가 나오면 터미널 2로 넘어갑니다.

**터미널 2: 검증 실행** (새 터미널)
```bash
source ~/doosan_ros2_ws/install/setup.bash
```
```bash
python3 ~/doosan_ros2_ws/scripts/pp_scene_check.py
```
마지막에 `✅ 문제 없음` 이 나오면 실행해도 됩니다. ⚠ 줄이 나오면 내용대로 pick / place 를 다시 저장합니다.

### 2) RViz로 실물 로봇 같이 보기 (DRFL GUI와 함께)
1)의 가상 모드 MoveIt은 먼저 끕니다(같은 관절 토픽을 써서 화면이 섞임). 띄우는 순서는 상관없습니다.

**터미널 1: RViz 미러** (켜 둡니다)
```bash
source ~/doosan_ros2_ws/install/setup.bash
```
```bash
ros2 launch m0609_mirror mirror.launch.py
```

**터미널 2: 제어 GUI** (새 터미널)
```bash
cd ~/doosan-m0609-drfl
```
```bash
./m0609_gui/build/m0609_gui
```
GUI에서 조그·그리퍼·Pick & Place를 하면 RViz의 로봇과 그리퍼 색(초록 열림 / 주황 닫힘 / 파랑 잡음)이 같이 바뀝니다.

### 책상
**책상은 실습실 공통 고정값입니다.** 모든 실습 책상이 같으므로 `scripts/pp_scene.yaml` 의 `desk.plane`(실제 책상 면)과
`table.top_z: 11`(충돌 모델 박스 윗면)을 그대로 쓰고, 학생은 책상을 측정하지 않습니다. 책상이 바뀐 경우에만 아래
"테이블 측정" 절차로 다시 재서 두 값을 고칩니다.

---

# 참고 기록 (개발·검증 과정)

## 수동 빌드 (`setup_doosan_robot2.sh` 가 하는 일)
```bash
sudo apt-get install -y libpoco-dev ros-humble-ros2-controllers dbus-x11
cd ~/doosan_ros2_ws && source /opt/ros/humble/setup.bash
colcon build --packages-up-to dsr_msgs2 dsr_common2 dsr_description2 dsr_hardware2 dsr_controller2 dsr_bringup2 dsr_moveit_config_m0609 \
             --cmake-args -DDRCF_VER=2 -DCMAKE_BUILD_TYPE=Release
```
- 소스: `src/doosan-robot2` (github DoosanRobotics/doosan-robot2, humble 브랜치 8653b04)
- Gazebo / MuJoCo 패키지는 빌드하지 않음
- 에뮬레이터 이미지: `docker pull doosanrobot/dsr_emulator:3.0.1` (가상 모드에서 자동 실행/종료)

## 가상 모드 (MoveIt + RViz) — ✅ 확인
```bash
cd ~/doosan_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=m0609 host:=127.0.0.1
```
다른 터미널에서 MoveIt 관절 목표 시험:
```bash
cd ~/doosan_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
python3 scripts/moveit_joint_test.py 30 20 60 0 0 0     # J1~J6 [deg], 속도/가속 20%
```
결과: 계획·실행 성공, 최종 관절 오차 0.05° 이내.

## 주의: 가상 모드와 실물 모드는 실행 경로가 다름
`dsr_hardware2` write():
- 가상: 매 주기 `amovej()` (에뮬레이터는 실시간 연결 미지원)
- 실물: `servoj_rt()` 실시간 스트리밍 (UDP 12347)

→ 가상 모드 성공이 실물(컨트롤러 2.10.1, 공식 지원은 2.12 이상) 동작을 보장하지 않습니다.
## 실물 1단계: RViz 연결 (명령 없음) — ✅ 확인 2026-11-13
```bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real model:=m0609 host:=192.168.137.100 port:=12345
```
- 이 launch는 `joint_state_broadcaster` + `dsr_controller2`만 띄움. `dsr_controller2`는 위치 명령 인터페이스를 쓰지 않으므로
  `servoj_rt`가 나가지 않음 → 로봇이 움직이지 않음 (코드 확인)
- 드라이버가 스스로 제어권 강제 획득 + Servo ON (SAFE_OFF에서도 시작 가능)
- 컨트롤러 2.10.1(GF02100101)에서: RT 스트림 연결·시작 성공, `/dsr01/joint_states` 100 Hz,
  관절값이 DRFL 값과 0.001° 단위까지 일치
- 네임스페이스는 `dsr01` (토픽: `/dsr01/joint_states`)

## 실물 2단계: MoveIt 실물 이동 — ✅ 확인 2026-11-13
```bash
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.137.100 port:=12345
# 다른 터미널
python3 scripts/moveit_rel_joint.py 6 5 0.1     # J6 +5°, 속도/가속 10%
python3 scripts/moveit_rel_joint.py 6 -5 0.1    # 되돌리기
```
- 결과: 계획·실행 성공 (궤적 11점, 0.94초), 목표 오차 0.01° 이내, 왕복 후 원래 자세로 복귀
- 컨트롤러 2.10.1에서 `servoj_rt` 실시간 스트리밍으로 실제 이동 확인 (공식 지원 2.12 미만이지만 동작)
- MoveIt launch 기동 전후 관절 변화 0.001° 수준 (시작 시 튀지 않음)

### 로컬 수정 (안전)
`src/doosan-robot2/dsr_controller2/config/dsr_controller2.yaml` 의 `dsr_moveit_controller`에
`set_last_command_interface_value_as_state_on_activation: false` 추가.
드라이버의 위치 명령 초기값이 0 rad이라, JTC 기본값(true)이면 활성화 순간 "0° 자세 유지"를 목표로 삼을 수 있음.
false로 두면 실제 측정 관절값에서 시작. 수정 후 `colcon build --packages-select dsr_controller2` 필요.

## 테이블 측정 (책상이 바뀐 경우에만) + MoveIt 장면 검증 (2026-11-13)
- `scripts/table_points.py`: 직접 교시로 손가락 끝을 테이블에 대고 기록 (읽기 전용 DRFL `tools/flange_pose`, 제어권 불필요)
  - 측정 4점 → 측정면 기울기 1.5°, 측정점 중심 Z ≈ 6 mm (툴 140 + 띄운 간격 15 mm 기준), 결과 `table_points.json`
- `scripts/pp_scene.yaml`: 장면 설정 (책상 고정값 `desk.plane`, 평평한 책상 박스 윗면 11 mm / 그리퍼 / 직육면체 20×20 × 60·50·40 mm / 고정 물체)
- `scripts/pp_scene_check.py`: 장면 적용 + 티칭 home/pick/place 검증 (로봇 이동 없음, 가상 모드 MoveIt에서 실행)
  - 손가락 끝 높이, 접근점 IK, 자세별 충돌, 관절 경로 / 직선 경로 계획, 물체를 잡은 구간
  - 음성 시험: pick 아래 40 mm(손가락 끝이 테이블 아래)에서 `table ↔ gripper` 충돌 검출 확인
- 공식 `dsr_moveit_config_m0609` 의 demo.launch.py 는 없는 패키지(m0609_moveit_config)를 참조해 실행 불가 → 가상 모드 사용

## RViz 미러 구현 메모 (2026-11-13)
```bash
cd ~/doosan_ros2_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch m0609_mirror mirror.launch.py            # ip:=192.168.137.100 기본
```
- `src/m0609_mirror`: `drfl_mirror` 노드가 DRFL 모니터링을 **읽기 전용**으로 받아 `/joint_states`(50 Hz)와
  `/gripper_state`(open/closed/holding/middle)를 발행. 제어권 요청·명령 없음 → DRFL GUI와 동시에 사용 가능
- `scene_markers.py`: 책상 박스, 그리퍼(손가락 영역 색: 초록=열림, 주황=닫힘, 파랑=물체 잡음), 잡은 물체(빨강),
  티칭 home/pick/place(손가락 끝 위치), Pick & Place 경로(노란 선, 수직 접근 + 이동 높이). 2초마다 파일을 다시 읽음
- 관절값은 DRFL 값과 0.01° 단위까지 일치 확인
- ROS 드라이버(dsr_bringup2)나 가상 모드와 동시에 띄우지 말 것 (`/joint_states` 충돌)
