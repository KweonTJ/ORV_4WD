#!/usr/bin/env bash
# ORV MuJoCo launcher. Works from any current directory, including via symlink.
if [[ "${BASH_SOURCE[0]}" != "$0" ]]; then
    printf '이 파일은 source 대신 bash 또는 ./run_simulation.sh로 실행하세요.\n' >&2
    return 1
fi
set -eo pipefail

orv_script_path=$(readlink -f -- "${BASH_SOURCE[0]}")
orv_source_dir=$(dirname -- "$orv_script_path")
orv_workspace_dir=$(dirname -- "$orv_source_dir")
orv_build=false
orv_launch_args=()

usage() {
    cat <<'HELP'
ORV 4WD MuJoCo 시뮬레이션

사용법: ./run_simulation.sh [옵션] [ROS launch 인자...]

  인자 없음     차량·모터 상태·주행 제어를 통합 창으로 실행
  --course      경사판·장애물 코스 실행
  --headless    GUI와 뷰어 없이 물리 시뮬레이션 실행
  --build       필요한 ROS 패키지를 다시 빌드한 뒤 실행
  -h, --help    이 도움말 표시

예시:
  ./run_simulation.sh
  ./run_simulation.sh --course
  ./run_simulation.sh --headless
  ./run_simulation.sh --build bridge:=true
  ./run_simulation.sh rviz:=true

ROS Humble과 워크스페이스 환경을 자동으로 불러옵니다.
실행 파일이 아직 설치되지 않았다면 먼저 colcon으로 빌드합니다.
통합 창을 닫거나 터미널에서 Ctrl+C를 누르면 종료합니다.
HELP
}

fail() {
    printf 'ORV 실행 오류: %s\n' "$*" >&2
    exit 1
}

while (($#)); do
    case "$1" in
        -h|--help) usage; exit 0 ;;
        --build) orv_build=true ;;
        --course) orv_launch_args+=(terrain:=course) ;;
        --headless) orv_launch_args+=(viewer:=false gui:=false) ;;
        --) shift; orv_launch_args+=("$@"); break ;;
        *) orv_launch_args+=("$1") ;;
    esac
    shift
done

[[ -r /opt/ros/humble/setup.bash ]] || fail 'ROS 2 Humble 설치를 확인하세요: /opt/ros/humble/setup.bash'
[[ -f "$orv_source_dir/orv_mujoco/package.xml" ]] || fail 'run_simulation.sh는 ORV 워크스페이스의 src 폴더에 있어야 합니다.'
# ROS/colcon setup scripts may refer to unset variables, so do not enable nounset.
source /opt/ros/humble/setup.bash
cd -- "$orv_workspace_dir"

if [[ "$orv_build" == true || ! -r install/local_setup.bash ||
      ! -x install/orv_mujoco/lib/orv_mujoco/dashboard ]]; then
    command -v colcon >/dev/null || fail 'colcon이 필요합니다. python3-colcon-common-extensions 설치를 확인하세요.'
    printf 'ORV 시뮬레이션 패키지 빌드: %s\n' "$orv_workspace_dir"
    colcon build --symlink-install --packages-up-to orv_mujoco
fi
[[ -r install/local_setup.bash ]] || fail '워크스페이스 환경 파일이 없습니다. --build 옵션으로 다시 실행하세요.'
source install/local_setup.bash

if ! /usr/bin/python3 -c 'import mujoco; import rclpy; import orv_mujoco' >/dev/null 2>&1; then
    printf 'MuJoCo 및 ROS Python 환경을 확인하세요. MuJoCo 설치 명령:\n' >&2
    printf '  /usr/bin/python3 -m pip install --user -r %q\n' "$orv_source_dir/orv_mujoco/requirements.txt" >&2
    exit 1
fi

printf 'ORV MuJoCo 실행 · 종료: 창 닫기 또는 Ctrl+C\n'
# Replace this shell so terminal signals reach ros2 launch and its child nodes.
exec /opt/ros/humble/bin/ros2 launch orv_mujoco simulation.launch.py "${orv_launch_args[@]}"
