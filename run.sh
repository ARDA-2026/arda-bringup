#!/usr/bin/env bash
# ARDA bringup 원스톱 빌드+실행 스크립트.
#
# 센서(tracker_node)/알고리즘(drift_node)/웹 브릿지(web_bridge_node)를
# 따로 실행할 필요 없이 이 스크립트 하나로 (선택) 빌드 + 실행이 끝난다:
#   ./run.sh            빌드는 건너뛰고 바로 실행 (평소 반복 실행용)
#   ./run.sh --build    venv 파이썬으로 colcon build 먼저 하고 실행
#
# venv는 activate할 필요 없다 — `venv/bin/python3 colcon build`로 빌드하면
# install/arda_bringup/lib/arda_bringup/의 각 노드 실행 스크립트 셔뱅이 이미
# venv 인터프리터를 가리키도록 설치되기 때문이다 (README "venv를 매번
# activate 안 해도 되는 이유" 참고). 여기서는 ROS2가 패키지를 찾기 위한
# install/setup.bash 오버레이만 source한다.
set -e
cd "$(dirname "$0")"

if [[ "$1" == "--build" ]]; then
  shift
  echo "[run.sh] venv 파이썬으로 colcon build 중..."
  venv/bin/python3 "$(command -v colcon)" build --packages-select arda_bringup
fi

source install/setup.bash
exec ros2 launch arda_bringup bringup.launch.py "$@"
