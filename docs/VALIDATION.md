# ORV 4WD 검증 기록

검증 환경: Ubuntu 호스트, ROS 2 Humble, Python 3.10. Raspberry Pi 4B와 실제 모터에서의 동작은 별도 검증 대상입니다.

2026-10-04 사용자 확인: 바퀴 지름 88 mm, 좌우 바퀴 바깥쪽 끝 사이 전체 폭 245 mm, 좌우 바퀴 폭 각각 35 mm. 실물·모의 설정과 URDF 기본 반경은 0.044 m입니다. 전체 폭 0.245 m에서 바퀴 폭 0.035 m를 빼서 URDF와 실물·모의 설정의 중심 간격을 0.210 m로 반영했습니다.

2026-10-04 추가 사용자 확인: 차체 전면부터 후면까지 길이 290 mm, 차체 자체 높이 67 mm, 지면에서 차체 바닥까지 높이 21 mm. URDF와 display launch 기본값을 각각 0.290 m, 0.067 m, 0.021 m로 반영했습니다. 차체 중심 Z는 0.0545 m, 상단은 0.088 m입니다. 앞뒤 바퀴 중심 간격은 미확정이며 시각화용 0.200 m를 유지합니다.

## 완료

- 상세 메시: 제공 STEP에서 차체 추출, 사진 기반 일반 타이어·스포크·트레드 및 전면/측면 부품 생성. COLLADA 좌표/인덱스, 차체 외곽 290 × 155 × 67 mm, 바퀴 반경 44 mm·폭 35 mm 검증
- 상세 URDF `check_urdf` 통과, `orv_description` 빌드·메시 설치 경로 확인, 실행 중 설명과 4개 바퀴 TF 위치 확인
- 바퀴별 `chassis_link → *_axle_link → *_wheel_link` 연결과 은색 축 형상 추가. `check_urdf` 통과, 4개 축이 차체 측면부터 바퀴 중심까지 연결됨을 확인. 실행 중 고정·회전 TF 및 바퀴 중심 위치 유지 확인. 연결축 지름 12 mm는 시각화용 가정값
- 차체 길이 290 mm, 높이 67 mm 생성 결과와 실행 중 `/robot_description` 갱신 확인. 높이 변경 시 차체 바닥 높이 유지 확인. 앞뒤 바퀴 중심 간격은 기존 가정값 200 mm 유지
- 지상고 21 mm 생성 결과와 실행 중 `/robot_description`, `/tf_static` 확인: 차체 중심 Z 54.5 mm, 상단 88 mm. 차체 높이 변경 시 지상고 유지 및 바퀴 4개의 지면 접촉 확인
- Xacro 생성 결과에서 바퀴 4개 지름 88 mm, 폭 35 mm, 중심 간격 210 mm, 외측 전체 폭 245 mm 및 바닥 접촉 높이 확인. 바퀴 폭 변경과 중심 간격 재지정도 확인
- `colcon build --symlink-install`: 5개 패키지 빌드
- Python 제어·프로토콜·실제 PTY 시리얼·도메인 설정 테스트 35개 통과
- CRC 기준 벡터, 잘린 프레임, 손상 프레임 뒤 복구, 길이 제한
- 설정 범위/비정상 수치 거부, 채널 매핑, 좌우 비율을 유지하는 속도 제한
- 전진·후진·제자리 회전 오도메트리, signed 카운트 rollover
- 명령 만료, 소스 경합 거부, 구동 중 설정 거부
- 연결 끊김 후 재연결 및 UNO 설정 소실 후 재적용, 자동 재구동 금지
- 하드웨어 확인 플래그 및 치수 미설정 시 ROS 주행 차단
- PTY 양방향 통신, 중복 포트 소유 방지, 포트 끊김 감지
- 모의 launch 통합: ROS arm 서비스 → cmd_vel → joint_states/odom, 명령 만료 정지
- 공식 Humble `domain_bridge` 0.5.0 실행 파일로 두 localhost 테스트 도메인 간 전달 검증: PC `/orv/cmd_vel` → 차량 `/cmd_vel`, 차량 상태·오도메트리·관절 상태·TF → PC `/orv/...`
- 브릿지 시작 후 구독한 노드에서도 `transient_local` 고정 TF와 전체 URDF 수신. PC의 원래 `/odom`에는 차량 데이터가 전달되지 않는 것 확인
- mock 차량에서 브릿지를 종료한 뒤 PC가 계속 명령을 발행해도 차량 watchdog으로 구동 해제됨을 확인
- 도메인 설정 추가 후 기존 ROS/HTTP 통합 검증 재통과. 브릿지 테스트 및 차량 launch 정상 종료 확인
- HTTP 통합: PID 변경과 적용 확인 → 개별 바퀴 회전 → 전체 정지
- 적용 확인된 설정의 임시 YAML 저장 및 저장된 PID 값 재확인
- launch 종료 시 중복 SIGINT 처리와 전체 프로세스 정상 종료 (Traceback/ERROR 없음)
- Qt GUI 입력 설정 왕복 및 화면 렌더링 (Linux PyQt5, offscreen)
- GUI 모터 시험 버튼 hold/release 정지와 비활성 상태 버튼 차단
- UNO 보드용 컴파일: Arduino AVR core 1.8.6, `arduino:avr:uno`, AVR GCC 7.3.0
- UNO 플래시 10,536 / 32,256 bytes (32%), 전역 RAM 739 / 2,048 bytes (36%). 스택 여유량은 실물에서 추가 검증 필요
- `colcon test-result`: 35 tests, 0 errors, 0 failures, 0 skipped

UNO 컴파일에는 Arduino CLI 1.5.2-rc.1을 임시 도구 경로에서 사용했습니다. 경고는 AVR 코어의 `new.cpp`에 있는 사용하지 않는 인자에 한정됐습니다. 산출물은 워크스페이스 기준 `build/orv_uno/orv_uno.ino.hex`에 있습니다.

도메인 브릿지는 개발 PC 사용자 경로의 공식 Debian 패키지 추출본으로 검증했습니다. 2026-10-04 브릿지 통합 로그: `/tmp/orv_bridge_check_j6i6_ryb`, 기존 ROS/HTTP 통합 로그: `/tmp/orv_integration_qjw79e1s`. 테스트는 실제 운용 도메인과 다른 localhost 도메인 및 mock 장치만 사용했습니다.

## 미검증

- 실물 실드 버전, 엔코더 PPR/CPR, 모터·엔코더 방향, M번호와 바퀴 위치
- 실제 UNO 업로드와 USB 연결, 4개 모터 최대 회전수에서의 인터럽트 부하
- 하중에 따른 유효 바퀴 반경, 회전 슬립에 따른 유효 중심 간격, 하중별 PID와 최대 속도
- Raspberry Pi ARM64에서의 빌드·실행
- 실제 Pi ↔ PC LAN에서 DDS 검색, 방화벽·멀티캐스트 및 domain_bridge 통신
- Windows/PySide6 실행 및 Windows 실행 파일 패키징
- 서보 제어, Nav2 구성, `ros2_control` 플러그인

GUI 그래프와 mock 통합 테스트의 성공은 실제 하드웨어 주행 검증을 의미하지 않습니다.
