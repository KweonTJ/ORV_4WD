# ORV MuJoCo 디지털 트윈

`orv_mujoco`는 두 가지 실행을 제공합니다.

- **물리 시뮬레이션:** ROS/GUI 명령 → 모터 PID·PWM → 토크 → 바퀴/지면 접촉 → 가상 엔코더 → 기존 ROS 드라이버. USB를 열지 않습니다.
- **실차 미러:** domain_bridge로 받은 실제 차량의 오도메트리·바퀴 관절 상태를 MuJoCo 모델에 반영합니다. 주행 명령을 발행하지 않습니다. 이 모드에서는 물리 적분 없이 수신한 자세를 표시합니다.

![MuJoCo에서 렌더링한 ORV 물리 모델](images/orv_mujoco.png)

위 이미지는 생성형 이미지가 아닌 MuJoCo 장면의 실제 렌더입니다. RViz용 상세 메시를 OBJ로 변환해 차체 구멍·전면 패널·일반 타이어 형상을 유지했습니다.

## 설치와 실행

Ubuntu 22.04 / ROS 2 Humble / Python 3.10 / MuJoCo 3.10.0으로 검증했습니다. 시뮬레이션은 Ubuntu PC에서 실행하는 구성이 기본입니다. 기존 Raspberry Pi의 실차 드라이버에는 MuJoCo를 설치할 필요가 없습니다.

```bash
cd /home/ktj/ORV_4WD
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r src/orv_mujoco/requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch orv_mujoco simulation.launch.py
```

반복 실행에는 [`run_simulation.sh`](../run_simulation.sh)를 사용합니다. ROS 환경 설정과 워크스페이스 이동을 자동 처리하며, 빌드 결과가 없으면 필요한 패키지를 빌드합니다. MuJoCo 등 외부 의존성은 위 설치 과정에서 준비합니다.

```bash
/home/ktj/ORV_4WD/src/run_simulation.sh                # 통합 창
/home/ktj/ORV_4WD/src/run_simulation.sh --course       # 경사판·장애물 코스
/home/ktj/ORV_4WD/src/run_simulation.sh --headless     # 화면 없이 실행
/home/ktj/ORV_4WD/src/run_simulation.sh --build        # 재빌드 후 실행
/home/ktj/ORV_4WD/src/run_simulation.sh bridge:=true   # 선택적으로 domain_bridge 실행
```

각 명령은 별도의 실행 예입니다. 현재 PC의 워크스페이스 루트에도 `run_simulation.sh` 링크가 있어 `/home/ktj/ORV_4WD/run_simulation.sh`로 실행할 수 있습니다. `--help`로 도움말을 확인하고, 창 닫기 또는 `Ctrl+C`로 종료합니다.

기본 실행은 **하나의 통합 창**을 최대화해 엽니다. 왼쪽에 MuJoCo 차량·실제 물리 위치·RPM 그래프, 오른쪽에 네 모터의 엔코더/RPM/PWM·오도메트리·주행 버튼이 표시됩니다. 오른쪽 `PID / 엔코더 설정` 탭에서도 차량 화면과 상태 표는 계속 보입니다.

![MuJoCo 통합 화면 실제 캡처](images/orv_mujoco_dashboard.png)

처음에는 비활성이므로 `GUI 구동 활성화` 후 전진·회전·모터 시험 버튼을 누릅니다. 버튼을 놓거나 창이 비활성화되면 출력이 해제되며 차량은 마찰에 따라 관성으로 더 굴러갈 수 있습니다. 바퀴가 멈춘 뒤 다시 활성화하거나 PID·최대 RPM·PWM·가속 제한을 변경합니다. `차량 위치 초기화`는 비활성 상태에서 차량 자세·엔코더·오도메트리를 초기화합니다.

차량 화면의 왼쪽 드래그는 회전, 오른쪽 드래그는 시점 이동, 휠은 확대/축소입니다. 더블 클릭 또는 `차량 따라가기 / 시점 복원`으로 추적 시점을 복원합니다. 화면 사이 구분선을 끌어 차량/제어 패널의 비율을 조절할 수 있습니다.

통합 창은 ROS 콜백·물리 상태·제어를 같은 Qt 스레드에서 처리하고 기존 드라이버의 명령 검증을 재사용합니다. 별도 GUI/클라이언트용 시뮬레이션 API `http://127.0.0.1:8766`도 유지됩니다. Linux 통합 화면은 EGL 오프스크린 렌더를 사용하므로 GLFW 뷰어 창이 추가로 열리지 않습니다.

기본 시뮬레이션 도메인은 실차/조종 PC 도메인과 별도이며, launch에서 실사용 도메인과 같은 값을 지정하면 거부합니다. 시뮬레이터는 localhost 통신을 사용합니다. 실제 차량 API의 기본 포트와도 구분했습니다.

```bash
# 경사판·낮은 턱·원통 장애물을 포함한 코스
ros2 launch orv_mujoco simulation.launch.py terrain:=course

# 뷰어 없이 계산만 실행
ros2 launch orv_mujoco simulation.launch.py viewer:=false gui:=false

# 제어 패널 없이 MuJoCo 네이티브 뷰어만 실행
ros2 launch orv_mujoco simulation.launch.py gui:=false

# RViz도 함께 표시
ros2 launch orv_mujoco simulation.launch.py rviz:=true
```

각 명령은 별도의 실행 예이며 같은 도메인/포트로 중복 실행하지 않습니다. 통합 창 또는 네이티브 뷰어를 닫으면 해당 launch의 시뮬레이터·상태 발행기를 함께 종료합니다. `viewer:=false gui:=true`는 기존 튜너만 별도로 열며, 이 경우 튜너를 닫아도 헤드리스 시뮬레이터는 유지됩니다. `rviz:=true`는 추가 RViz 창을 여는 선택 사항입니다.

## ROS 주행과 상태

시뮬레이션 터미널의 CLI 도메인은 다음처럼 설정합니다. 커스텀 `domain_id:=...`로 실행했다면 같은 값으로 CLI 환경을 맞추세요.

```bash
source /opt/ros/humble/setup.bash
source /home/ktj/ORV_4WD/install/setup.bash
export ROS_DOMAIN_ID="$(ros2 run orv_mujoco domain_id)"
export ROS_LOCALHOST_ONLY=1
ros2 topic echo /orv/status --once
```

GUI 구동을 해제한 뒤, 같은 시뮬레이션 도메인의 한 터미널에서 명령을 먼저 발행하고 다른 터미널에서 ROS 구동을 활성화합니다. 비활성 상태의 명령은 무시합니다.

```bash
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.06}, angular: {z: 0.0}}'
```

```bash
ros2 service call /orv/arm std_srvs/srv/SetBool '{data: true}'
ros2 service call /orv/stop std_srvs/srv/Trigger '{}'
```

마지막 정지 명령은 시험을 마칠 때 호출하세요. 주행 명령이 만료돼도 기존 watchdog으로 출력이 해제됩니다. `/orv/status`의 `mode`는 `mujoco`입니다.

| 시뮬레이션 도메인의 인터페이스 | 의미 |
|---|---|
| `/cmd_vel`, `/orv/arm`, `/orv/stop` | 기존 주행 명령·제어권·정지 |
| `/joint_states`, `/orv/status` | 물리 바퀴 관절에서 만든 가상 카운트·RPM·PWM |
| `/odom`, `/tf` | 기존 ROS 드라이버의 엔코더 기반 추정 |
| `/orv/sim/ground_truth` | 물리 엔진이 계산한 위치·자세·속도, `sim_world` 기준 |
| `/orv/sim/metrics` | 실시간 배율, 접촉 수, 버린 지연 시간, 위치 오차, 가정 질량 |
| `/orv/sim/reset` (`std_srvs/srv/Trigger`) | 비활성 상태에서 물리 위치·바퀴 카운트·오도메트리 초기화 |

```bash
ros2 service call /orv/sim/reset std_srvs/srv/Trigger '{}'
```

`/orv/reset_odometry`는 기존과 같이 **추정 원점만** 바꿉니다. 물리 위치도 되돌리려면 `/orv/sim/reset`을 사용하세요. 추정 원점만 바꾼 후에는 `odometry_position_error_m`의 원점이 서로 다르므로 보정 오차로 해석하지 마세요.

기존 오도메트리·RViz TF는 평면 주행 추정입니다. 경사판 위의 높이·롤·피치까지 포함한 자세는 MuJoCo 화면과 `ground_truth`에서 확인합니다.

시뮬레이션 시간은 물리 스텝으로 진행하고 ROS 메시지 시각은 호스트 시계를 사용합니다. 이 실행은 실시간 제어용이며 `/clock`을 발행하거나 `use_sim_time`을 사용하지 않습니다. 과도한 지연은 제한된 스텝만 따라잡고 metrics에 기록합니다. 긴 멈춤 뒤 이전 주행 명령을 한꺼번에 재생하지 않습니다.

## 조종 PC 도메인으로 브릿지

```bash
ros2 launch orv_mujoco simulation.launch.py bridge:=true
```

PC에서는 **`/orv_sim/...`** 토픽을 사용합니다. 실제 차량의 `/orv/...`와 구분합니다.

| PC 토픽 | 시뮬레이션 토픽 | 방향 |
|---|---|---|
| `/orv_sim/cmd_vel` | `/cmd_vel` | PC → 시뮬레이터 |
| `/orv_sim/odom` | `/odom` | 시뮬레이터 → PC |
| `/orv_sim/joint_states` | `/joint_states` | 시뮬레이터 → PC |
| `/orv_sim/status` | `/orv/status` | 시뮬레이터 → PC |
| `/orv_sim/tf`, `/orv_sim/tf_static` | `/tf`, `/tf_static` | 시뮬레이터 → PC |
| `/orv_sim/robot_description` | `/robot_description` | 시뮬레이터 → PC |
| `/orv_sim/ground_truth`, `/orv_sim/metrics` | `/orv/sim/ground_truth`, `/orv/sim/metrics` | 시뮬레이터 → PC |

브릿지 설정은 `orv_mujoco/config/domain_bridge.yaml`입니다. launch는 시뮬레이션 도메인 및 기존 PC 도메인 설정을 읽어 실제 브릿지 실행 인자에 적용합니다. 서비스는 브릿지하지 않으므로 ROS의 ARM·STOP·reset은 시뮬레이션 도메인 터미널에서 호출합니다. 통합 창의 버튼은 내부 제어 경로를, 별도로 실행한 튜너는 HTTP를 사용합니다.

## 실제 차량을 화면에 동기화

기존 차량 bringup과 차량용 domain_bridge가 실행 중인 PC에서:

```bash
ros2 launch orv_mujoco mirror.launch.py
```

미러는 설정된 PC 도메인의 `/orv/odom`, `/orv/joint_states`만 구독해 모델을 갱신합니다. 따라서 동봉 조종기로 실차를 움직여도 통합 펌웨어가 엔코더를 전송하면 모델이 따라 움직입니다. 구동 명령·ARM·GUI 서버는 만들지 않습니다. 데이터가 0.5초 이상 없으면 모델을 마지막 자세에 멈추고 로그에 만료 상태를 표시합니다.

실차의 위치는 엔코더 추정값이며 외부 위치 센서로 검증된 절대 위치가 아닙니다. 실제 차량과의 연결은 아직 실물로 검증하지 않았고, 미러의 수신·표시는 시뮬레이션 토픽을 사용해 검증했습니다.

## 치수와 물리 파라미터

[`physics.yaml`](../orv_mujoco/config/physics.yaml)에 물리 형상을, [`driver.yaml`](../orv_mujoco/config/driver.yaml)에 PID·엔코더 보정을 저장합니다. 물리 파라미터 변경은 재실행해야 반영됩니다.

| 항목 | 값과 근거 |
|---|---|
| 바퀴 지름·폭 | 88 mm / 35 mm, 사용자 실측 |
| 좌우 중심 간격·바깥 폭 | 210 mm / 245 mm, 사용자 실측에서 계산 |
| 차체 길이·높이·지상고 | 290 mm / 67 mm / 21 mm, 사용자 실측 |
| 앞뒤 축 간격·차체 폭 | 200 mm / 155 mm, 기존 모델 가정 |
| 전체 질량·바퀴별 질량 | 2.5 kg / 0.09 kg, **임시값** |
| 관성 | 가정 질량과 박스/원통 공식으로 계산 |
| 모터 출력축 스톨 토크 | 1.2 N·m, **임시값** |
| 무부하 속도 | 107 RPM, 제공 자료의 정격 전압 기준 |
| 마찰·축 감쇠·회전부 관성 | **임시값**, 실험으로 보정 필요 |
| 가상 물리 엔코더 | 4320 count/rev, 잠정값 |

엔코더 보정값은 가상 엔코더 자체의 해상도와 별도입니다. `physics.yaml`의 `physical_encoder_cpr`가 가상 센서의 실제 해상도이고 GUI/드라이버의 `encoder_cpr`는 이를 해석하는 보정값입니다. 잘못 보정하면 실제 차량처럼 속도·거리 추정도 달라집니다. 물리 반경·간격 역시 GUI의 오도메트리 보정값과 별도입니다. `physical_motor_map`은 가상 배선, 드라이버의 `motor_map`은 해석·제어 매핑입니다.

바퀴는 Y축 회전 관절과 독립 토크 구동기를 사용합니다. MCU와 같은 20 ms PID·가속 제한·PWM 제한을 근사하고, 선형 PWM/역기전력 토크 모델을 사용합니다. 엔코더는 목표 속도가 아니라 물리 관절 회전각에서 양자화합니다. 4륜 스키드 스티어의 접촉·미끄러짐 때문에 회전 오도메트리와 실제 물리 자세가 달라질 수 있습니다. 초기 PID는 실제 하드웨어에 맞춘 값이 아니므로 코스 주행·제자리 회전 응답도 튜닝 대상입니다.

시뮬레이션은 AVR 명령어 수준 실행, 무선 수신기, 전류·발열·배터리 전압 강하, 타이어 탄성 모델을 포함하지 않습니다. 시뮬레이션 PID 튜닝 결과를 실차 검증 없이 그대로 적용하지 마세요. 현 단계는 **실측 형상을 반영한 동역학 보정 전 디지털 트윈**입니다.

## 검증과 메시 재생성

```bash
colcon test --packages-select orv_4wd orv_mujoco
colcon test-result --verbose
python3 src/tools/mujoco_integration_check.py \
  --simulation-domain "$ORV_TEST_VEHICLE_DOMAIN_ID" \
  --operator-domain "$ORV_TEST_OPERATOR_DOMAIN_ID"
python3 src/tools/mujoco_dashboard_check.py --test-domain "$ORV_TEST_GUI_DOMAIN_ID"
```

실제 실행 중인 도메인과 겹치지 않는 두 테스트 도메인을 먼저 지정합니다. 물리 정지 안정성, 질량·치수, 전후진, 회전 미끄러짐, PWM 상한에 따른 반응, watchdog, ROS/HTTP/브릿지, 실차 미러 경로를 검증합니다.

통합 창 검사는 별도 테스트 도메인 하나를 지정합니다. 1366×800 화면에서 네 모터의 값과 설정 표시, 실제 Qt 버튼 주행, 버튼 해제/창 비활성/창 종료 시 출력 해제, 주행 중 ROS 오도메트리 발행, PID 적용·차량 초기화·렌더 자원 해제를 검사합니다. 실차를 연결하지 않습니다.

RViz COLLADA 메시가 바뀌었을 때만 다음을 실행합니다. CAD 원본이나 추가 변환 도구는 필요하지 않습니다.

```bash
python3 src/orv_mujoco/scripts/convert_meshes.py
colcon build --symlink-install --packages-select orv_mujoco
```

변환 입력의 SHA-256과 재질별 OBJ 목록은 `orv_mujoco/meshes/manifest.json`에 기록합니다. 모델·접촉 및 뷰어 API는 [MuJoCo 공식 XML 문서](https://mujoco.readthedocs.io/en/stable/XMLreference.html)와 [Python 문서](https://mujoco.readthedocs.io/en/stable/python.html)를 참고했습니다.
