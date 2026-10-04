# V3 4WD 형상 모델

2026-10-04 사용자가 제공한 세 장의 로봇 사진과 `V3_car_UNO/Rubber wheel car.jpg`, `V3 3D model file/V3 car 3D model.STEP`를 사용했습니다. 사진의 외형을 참고한 시각화 모델이며, 제작용으로 역설계한 정밀 도면은 아닙니다.

## 반영한 형상

- 원본 STEP에서 상·하판, 상판의 실제 타공과 슬롯, 기울어진 전면 프레임, 내부 모터 형상을 추출했습니다.
- 원본은 메카넘 바퀴 버전입니다. 바깥쪽 바퀴 영역을 제거하고 일반 고무 바퀴로 교체했습니다.
- 사진에서 확인되는 넓은 전면 브래킷의 원형 구멍과 십자 슬롯, 세 개의 통풍 슬롯이 있는 측면 판, 회색 전면 인서트와 체결부를 추가했습니다.
- 바퀴는 둥근 숄더, 16개 스포크, 허브, 5열의 교차하는 미로 형태 트레드로 구성했습니다. 좌우 바퀴의 스포크 면이 바깥을 향합니다.
- 네 개의 연결축과 기존 회전 관절 이름을 유지했습니다. 메시 전체를 고정된 한 덩어리로 만들지 않았으므로 실제 엔코더 관절값으로 바퀴가 회전합니다.

## 치수와 가정

| 항목 | 적용값 | 근거 |
|---|---:|---|
| 차체 길이 | 290 mm | 사용자 실측 |
| 차체 높이 | 67 mm | 사용자 실측 |
| 차체 바닥 지상고 | 21 mm | 사용자 실측 |
| 바퀴 지름 / 폭 | 88 / 35 mm | 사용자 실측 |
| 좌우 바퀴 외측 전체 폭 | 245 mm | 사용자 실측 |
| 좌우 바퀴 중심 간격 | 210 mm | 245 − 35 |
| 앞뒤 바퀴 중심 간격 | 200 mm | 기존 가정값 유지 |
| 차체 최대 폭 | 155 mm | 시각화용 가정값 |
| 연결축 지름 | 12 mm | 시각화용 가정값 |

STEP 차체를 축별로 스케일 조정했으므로 원본 모터·작은 체결부의 치수가 실물과 일치한다고 보장하지 않습니다. 타이어 트레드, 스포크, 전면 인서트, 브래킷, 측면 통풍판과 볼트는 사진 기반 근사 형상입니다. 실제 조립품의 앞뒤 바퀴 중심 거리와 차체 폭을 추가로 재면 배치를 더 정확히 맞출 수 있습니다.

차체 메시 원점은 차체 외곽 상자의 중심입니다. URDF에서 중심 Z를 `ground_clearance + chassis_height / 2`로 배치합니다. 바퀴 메시 원점은 회전 중심이고 회전축은 Y입니다. 오른쪽 바퀴의 시각 형상만 X축으로 180도 돌리며, 구동 관절의 축·부호는 변경하지 않습니다.

## 파일과 재생성

- `orv_description/meshes/v3_chassis.dae`: 차체와 고정 외장, 39,397 삼각형
- `orv_description/meshes/rubber_wheel.dae`: 바퀴 한 개, 12,224 삼각형
- `orv_description/meshes/provenance.json`: 원본 STEP SHA-256, 변환 범위, 스케일, 형상 출처
- `orv_description/scripts/generate_meshes.py`: 재생성 스크립트

실행 시 필요한 메시를 패키지에 포함했습니다. 아래 작업은 메시를 다시 만들 때만 필요하며 CAD 라이브러리를 ROS 실행 환경에 설치할 필요는 없습니다.

```bash
cd /home/ktj/ORV_4WD
python3 -m venv --system-site-packages build/mesh_tools_venv
build/mesh_tools_venv/bin/python -m pip install cadquery-ocp==7.8.1.1.post1 numpy
build/mesh_tools_venv/bin/python src/orv_description/scripts/generate_meshes.py \
  --step '/home/ktj/Documents/카카오톡 받은 파일/V3_car_UNO/V3 3D model file/V3 car 3D model.STEP' \
  --cache build/vendor_car.brep
source /opt/ros/humble/setup.bash
colcon build --packages-select orv_description --symlink-install
```

변환 스크립트의 바퀴 제외 범위는 제공된 STEP의 좌표계에 맞춰 작성했습니다. 다른 STEP로 바꾸면 영역 선택을 검토하고 캐시 경로도 새로 지정해야 합니다. 원본 CAD는 사용자 제공 자료이며, 메시 생성 코드의 라이선스가 원본 CAD에 대한 별도 권리를 부여하지는 않습니다.

충돌 형상은 단순 박스·원통입니다. 실제 타공이나 트레드 충돌, 질량·관성, 접촉 마찰을 검증한 Gazebo 물리 모델은 아닙니다.
