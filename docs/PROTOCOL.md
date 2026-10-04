# ORV UNO 프로토콜 v1

115200 baud, 8N1, USB 시리얼. MCU는 ROS를 실행하지 않습니다. 호스트 드라이버 하나가 포트를 소유합니다. 원래 공급된 14바이트 UartJoystick 규약과 호환되지 않습니다.

## 프레임

| 오프셋 | 길이 | 내용 |
|---:|---:|---|
| 0 | 2 | ASCII `OR` (`4f 52`) |
| 2 | 1 | 버전 `01` |
| 3 | 1 | 메시지 종류 |
| 4 | 2 | sequence, uint16 little endian |
| 6 | 1 | payload 길이, 최대 64 |
| 7 | N | payload |
| 7+N | 2 | CRC-16/CCITT-FALSE, little endian |

CRC 범위는 버전 바이트부터 payload 끝까지입니다. poly=0x1021, init=0xffff, refin=false, refout=false, xorout=0. `123456789`의 CRC는 0x29b1입니다. 모든 다중 바이트 수는 little endian입니다. 부호 있는 값은 2의 보수입니다. 부분 프레임은 다음 바이트가 100 ms 내 도착하지 않으면 폐기합니다.

## 명령

| 종류 | payload | 의미 |
|---|---|---|
| 1 CONFIG | 54 bytes | 정지 상태에서 설정 적용 |
| 2 ARM | uint8, 0/1 | 구동 해제/활성화. 설정 완료 및 fault 없음 필요 |
| 3 DRIVE | uint8 mode + int16[4] | mode=1: RPM×100, mode=2: signed PWM |
| 4 STOP | 없음 | 구동 해제, 출력 0, 목표·적분 초기화 |
| 5 RESET | 없음 | 비활성 상태에서 MCU 카운트 초기화. 기본 호스트 GUI는 이 명령 대신 상대 영점 사용 |
| 6 REMOTE_CONFIG | 7 bytes | 통합 펌웨어 전용: 조종기 허용·모터 매핑·속도 상한 |
| 7 HEARTBEAT | 없음 | 통합 펌웨어 전용: Pi 연결 유지. 별도 ACK 없음 |
| 128 STATE | 41 / 49 bytes | 구형 / 통합 펌웨어 주기 상태 보고 |
| 129 ACK | uint8 kind, uint8 result | 해당 sequence에 대한 명령 응답 |

ACK result는 0=적용, 1=형식·범위·상태 오류, 2=활성화 조건 불충족, 3=하드웨어 fault입니다. 정상 DRIVE는 별도 ACK 없이 STATE의 command sequence로 확인합니다. DRIVE sequence는 직전 DRIVE/ARM보다 modulo-65536 범위에서 앞으로 진행해야 하며 중복은 watchdog을 갱신하지 않습니다.

CONFIG/ARM은 ACK가 없으면 같은 sequence로 최대 3회 재전송합니다. 동일 ARM sequence의 재전송은 진행 중인 목표를 초기화하지 않습니다. 재연결 및 UNO 재시작 후에는 CONFIG를 다시 적용하고, 사용자 ARM을 새로 요구합니다.

### CONFIG payload

| 오프셋 | 타입 | 값 |
|---:|---|---|
| 0 | uint16[4] | M1..M4 CPR, 1..60000 |
| 8 | uint16[12] | M1 Kp·Ki·Kd, M2 ..., 각각 실제 게인×100, 0..10000 |
| 32 | uint8[4] | 최대 PWM, 1..255 |
| 36 | uint8[4] | 최소 비영 PWM, 0..254, 최대 이하 |
| 40 | int8[4] | 모터 방향, +1/-1 |
| 44 | int8[4] | 엔코더 방향, +1/-1 |
| 48 | uint16 | 최대 RPM×100, 100..15000 |
| 50 | uint16 | 가속 RPM/s×100, 100..30000 |
| 52 | uint16 | MCU watchdog ms, 100..1000 |

게인은 선상에서 0.01 단위로 양자화됩니다. 반경·간격·ROS 명령 타임아웃은 호스트가 처리합니다. 모터 매핑은 ROS 주행 시 호스트가 처리하고, 조종기용 매핑은 REMOTE_CONFIG로 추가 전달합니다. 영구 저장은 Pi의 YAML에 수행합니다.

### REMOTE_CONFIG와 HEARTBEAT

REMOTE_CONFIG는 `uint8 enabled + uint8[4] motor_map + uint16 max_rpm_x100`입니다. 매핑은 FL, RL, FR, RR 순서의 M번호이며 1~4의 순열이어야 합니다. 상한은 100~15000이고, MCU에서 주 CONFIG의 최대 RPM과 함께 제한합니다. 모터 CONFIG 적용 후 정지·비활성 상태에서만 수락합니다. 새 CONFIG는 조종기 설정을 무효화합니다.

호스트는 STATE의 bit6 지원 표시를 확인한 경우에만 이 확장을 사용합니다. ACK를 받은 뒤 약 50 ms마다 HEARTBEAT를 보내며, 조종기 주행에도 기본 MCU watchdog 300 ms를 적용합니다. HEARTBEAT sequence는 이전 heartbeat보다 modulo-65536 범위에서 앞으로 진행해야 하며, 중복 프레임은 기한을 연장하지 않습니다. 조종기 패킷만 계속 들어와도 Pi heartbeat가 만료되면 정지합니다. 일반 ROS/GUI 주행은 기존 DRIVE watchdog으로 감시합니다.

### STATE payload

| 오프셋 | 타입 | 값 |
|---:|---|---|
| 0 | uint32 | MCU uptime ms, wrap 허용 |
| 4 | uint8 | bit0 armed, bit1 configured, bit2 fault, bit3 remote_active, bit4 remote_connected, bit5 remote_configured, bit6 remote_supported |
| 5 | uint16 | 마지막 적용 CONFIG sequence |
| 7 | uint16 | 마지막 적용 DRIVE/ARM sequence |
| 9 | int32[4] | 엔코더 방향 보정된 누적 카운트 |
| 25 | int16[4] | 실제 RPM×100 |
| 33 | int16[4] | 방향 보정 이전 논리 출력 PWM |
| 41 | int16[4] | 통합 펌웨어만: M1..M4 목표값. RPM 모드는 RPM×100, 호스트 PWM 모드는 PWM |

bit6이 있는 통합 STATE는 49 bytes입니다. 구형 STATE는 41 bytes이며 새 호스트는 두 형식을 모두 읽습니다. 새 통합 펌웨어는 확장 STATE를 해석하는 새 호스트와 사용해야 합니다. 수신기 상태 bit4는 PS2 응답 유효성으로 결정하며 RF 연결 상태를 직접 보장하지 않습니다.

조종기 활성 상태에서는 호스트 ARM/DRIVE를 거부합니다. STOP과 ARM=0은 어느 모드든 해제하며, 조종기로 다시 시작하려면 START·L1을 모두 놓은 정상 입력을 받은 뒤 중립에서 L1 + START를 누릅니다. telemetry와 엔코더 계산은 제어 주체와 무관하게 계속됩니다.

카운트 차이는 signed 32-bit wrap을 고려해 계산합니다. 리셋·설정 변경에는 기준 카운트를 새로 잡습니다. 통신이 오래 끊기면 이전 프레임으로 새 오도메트리를 만들지 않습니다.

## HTTP 인터페이스

기본 `127.0.0.1:8765`. 원격 bind에는 `ORV_API_TOKEN` 16자 이상이 필요하고 클라이언트는 `Authorization: Bearer <token>`을 보냅니다. GUI에 ROS 설치가 필요하지 않습니다.

| 요청 | JSON 본문 |
|---|---|
| GET `/api/status` | 없음 |
| POST `/api/arm` | `{}` — GUI 소스 활성화 |
| POST `/api/stop` | `{}` |
| POST `/api/twist` | `{"linear": 0.05, "angular": 0.0}` |
| POST `/api/wheels` | `{"mode": "rpm", "values": [10,10,10,10]}` |
| POST `/api/config` | 변경할 Config 필드와 값. 정지·비활성 상태에서만 허용 |
| POST `/api/reset` | `{}` — 상대 카운트 영점 및 오도메트리 |
| POST `/api/save` | `{}` — 적용 확인된 설정을 YAML로 저장 |

HTTP 성공은 요청 접수/호스트 검증을 의미합니다. MCU 적용은 status의 `config_applied`, `armed`를 확인합니다. 본문은 최대 16 KiB, 대기열은 최대 32개이며 처리 기한이 지난 요청은 적용하지 않습니다. 전체 정지는 어떤 주행 소스에서도 가능하고 GUI/ROS/조종기의 주행 권한은 상호 배타적입니다. 조종기 모드의 `source`는 `remote`이며 `armed`는 MCU 상태를 반영하고 `arm_requested`는 호스트 요청 여부만 반영합니다.
