# 동봉 무선 조종기 + ROS 통합 펌웨어

스케치: [`orv_uno_integrated.ino`](../orv_firmware/firmware/orv_uno_integrated/orv_uno_integrated.ino). Arduino IDE에서는 같은 폴더의 `ps2_receiver.h`도 함께 필요합니다. 보드는 **Arduino UNO**입니다. 기존 `orv_uno` 스케치는 USB 전용 버전으로 보존했습니다.

## 지원 수신기와 배선

제공된 V3/QGPMaker Arduino 예제의 `config_gamepad(13, 11, 10, 12, ...)`를 기준으로 구현했습니다. 조종기를 무선으로 사용하는 것과 UNO 쪽 수신기 인터페이스는 별개입니다. 이 구현은 **UNO 실드에 연결하는 PS2형 수신기**용이며, Pi에 직접 페어링하는 Bluetooth 게임패드나 UART Bluetooth 모듈용 펌웨어는 아닙니다. 실제 동봉 수신기의 모델·접속 방식은 아직 실물로 확인하지 않았습니다.

| 수신기 신호 | UNO 핀 |
|---|---|
| CLK | D13 |
| CMD | D11 |
| ATT / CS | D10 |
| DAT | D12 |

실드의 전용 소켓과 전원·GND 배선을 확인하세요. 엔코더는 기존 D2~D9, USB 시리얼은 D0/D1, 모터 실드는 I²C A4/A5를 사용합니다. USB 시리얼에 별도 Bluetooth UART 모듈을 동시에 연결하는 구성은 지원하지 않습니다.

수신기는 20 ms마다 한 번 읽습니다. 엔코더 인터럽트를 끄지 않으며, 응답이 없을 때 무한 대기하지 않습니다. 디지털 모드에서는 아날로그 모드 전환을 단계적으로 시도하고, 정상 아날로그 응답을 받기 전까지 조종기 주행을 허용하지 않습니다. 진동·서보·메카넘 횡이동은 지원하지 않습니다.

## 최초 준비

1. 통합 스케치를 UNO에 업로드하고 Pi 워크스페이스를 다시 빌드합니다. 실제 보드 업로드는 이번 개발 과정에서 수행하지 않았습니다.
2. 바퀴를 띄운 상태에서 기존 절차대로 모터 방향, 엔코더 방향, CPR, `motor_map`을 확인합니다. 모든 모드가 같은 보정값을 사용합니다.
3. 확인한 설정에 `hardware_confirmed: true`, `remote_enabled: true`를 지정합니다. 기본 실물 설정은 확인 플래그가 false이므로 조종기 주행도 차단됩니다.
4. Pi의 `bringup.launch.py`를 `mode:=hardware`, 실제 UNO 포트 및 보정 설정으로 실행합니다. 시작 시 출력은 비활성입니다.
5. `/orv/status`에서 `ready`, `remote_supported`, `remote_enabled`, `remote_connected`가 true인지 확인합니다. PC에서는 README의 domain_bridge를 실행해 같은 상태를 받습니다.

Pi는 모터 설정과 조종기 설정을 UNO RAM에 전송하고 약 50 ms마다 heartbeat를 보냅니다. **이 버전은 Pi 드라이버가 실행 중이어야 조종기로 주행할 수 있습니다.** Pi 없이 켠 직후의 독립 주행이나 EEPROM 설정 저장은 구현하지 않았습니다. USB/드라이버가 끊기면 조종기 모드도 MCU watchdog 시간(기본 300 ms) 내 구동 해제합니다.

## 조작

| 동작 | 조작 |
|---|---|
| 조종기 모드 시작 | 차량이 멈춘 상태에서 START·L1을 한 번 모두 놓고, 왼쪽 스틱 중립에서 **L1 + START** |
| 전진·후진 | **L1을 계속 누른 채** 왼쪽 스틱 위·아래 |
| 좌우 회전 | **L1을 계속 누른 채** 왼쪽 스틱 좌·우 |
| 정지·조종기 구동 해제 | **L1 놓기** |
| 어느 제어 모드에서든 정지 | **SELECT**, ROS `/orv/stop`, GUI 전체 정지 |
| ROS/GUI로 전환 | 조종기 구동 해제 → 바퀴 정지 → ROS/GUI 활성화 |
| ROS/GUI에서 조종기로 전환 | ROS/GUI 정지 → 바퀴 정지 → L1 + START |

제어권을 자동으로 빼앗지 않습니다. ROS/GUI가 활성화된 동안 L1 + START는 조종기 구동을 시작하지 않으며, 조종기 구동 중 호스트 ARM/DRIVE는 거부합니다. 조종기 모드에서는 ROS `/cmd_vel`을 무시합니다. SELECT와 호스트 STOP은 제어 주체와 관계없이 적용합니다.

버튼을 누른 채 부팅·재연결하거나 STOP 이후 같은 버튼을 계속 누르고 있어도 자동으로 다시 구동하지 않습니다. 다시 시작하려면 START·L1을 모두 놓은 정상 패킷을 수신해야 합니다. L1 해제 또는 잘못된 수신기 응답은 조종기 구동을 해제합니다. 호스트 모드에서는 수신기가 없더라도 ROS/GUI 제어가 가능합니다.

조종기는 RPM 제어를 사용합니다. `remote_max_rpm` 기본값은 15이며, 실제 상한은 `min(remote_max_rpm, max_rpm)`입니다. 좌우 목표 비율을 유지하며 제한하고, 기존 가속 제한·바퀴별 PID·PWM 제한을 적용합니다. GUI 설정 화면에서도 조종기 사용 여부와 최대 RPM을 조절할 수 있습니다. 설정 변경은 구동 해제·정지 상태에서만 가능합니다.

## 토픽과 상태

조종기 구동 중에도 USB 상태 보고와 엔코더 집계를 계속합니다. Pi는 실제 카운트 변화로 관절 상태·오도메트리·TF를 만들고, domain_bridge가 PC에 전달합니다.

| PC 토픽 | 내용 |
|---|---|
| `/orv/status` | `source: "remote"`, `armed`, 수신기 상태, 목표 RPM, 실제 카운트/RPM/PWM |
| `/orv/joint_states` | 바퀴 회전각·각속도 |
| `/orv/odom` | 엔코더 기반 위치·속도 |
| `/orv/tf` | 오도메트리와 바퀴의 동적 TF |

상태의 `arm_requested`는 호스트의 구동 요청 여부이므로 조종기 모드에서는 false여도 `armed`가 true일 수 있습니다. `remote_connected`는 **UNO가 유효한 PS2 응답을 받았다는 의미**입니다. 수신기 내부의 무선 링크 상태를 별도로 읽은 값은 아닙니다. `/cmd_vel`이나 `/joy`를 조종기 입력에서 자동 발행하지는 않습니다.

구형 41-byte STATE를 사용하는 USB 전용 펌웨어도 새 드라이버로 사용할 수 있지만 `remote_supported`는 false입니다. 통합 펌웨어는 확장 상태 메시지를 사용하므로 **새 드라이버와 함께** 사용하세요.

## 컴파일과 검증

```bash
arduino-cli core install arduino:avr@1.8.6
arduino-cli compile --fqbn arduino:avr:uno \
  /home/ktj/ORV_4WD/src/orv_firmware/firmware/orv_uno_integrated
```

워크스페이스 빌드 후, 사용하지 않는 테스트 도메인을 지정해 수동 입력의 ROS 발행 경로를 확인합니다.

```bash
source /opt/ros/humble/setup.bash
source /home/ktj/ORV_4WD/install/setup.bash
python3 /home/ktj/ORV_4WD/src/tools/remote_integration_check.py \
  --domain "$ORV_TEST_VEHICLE_DOMAIN_ID"
```

실제 스케치의 상태 머신을 하드웨어 스텁으로 실행하는 테스트:

```bash
cd /home/ktj/ORV_4WD
g++ -std=c++17 -Wall -Wextra -Werror -D__AVR_ATmega328P__ \
  -I src/orv_firmware/test/stubs src/orv_firmware/test/integrated_test.cpp \
  -o build/integrated_firmware_test
./build/integrated_firmware_test
```

컴파일과 소프트웨어 테스트는 실제 수신기 타이밍·전압, RF 연결 단절 시 수신기의 동작, 엔코더 최대 인터럽트 부하를 검증하지 않습니다. 일부 수신기는 무선 단절 후에도 마지막 버튼 값을 정상 패킷처럼 반환할 수 있으므로 **바퀴를 띄운 상태에서 조종기 전원을 껐을 때 출력이 멈추는지 실물 검증이 필요합니다.** 기존 모터 보정 및 물리 정지 수단 확인 절차를 따르세요.
