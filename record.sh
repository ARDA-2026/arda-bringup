#!/usr/bin/env bash
# 레이더/열화상/서보 센서 타이밍 분석용 rosbag2 녹화.
#
# tracker_node가 이미 떠 있어야 한다(다른 터미널에서 ./run.sh). 이 스크립트는
# 그 위에서 관련 토픽만 골라 녹화한다 — 노드를 새로 띄우지 않는다.
#
# 사용법:
#   ./record.sh              bags/<타임스탬프>/ 에 녹화 (Ctrl+C로 종료)
#   ./record.sh my_test      bags/my_test/ 에 녹화 (이미 있으면 ros2 bag이 거부함)
#
# 재생/분석:
#   ros2 bag info bags/<이름>          토픽별 메시지 개수·기간 요약
#   ros2 bag play bags/<이름>          그대로 재생 (다른 노드로 흘려보내 재현)
#   ros2 topic echo ... 를 play와 같이 쓰면 시각별 값을 다시 볼 수 있음
#
# dwell_seconds/thermal_pending_timeout 같은 타이밍 파라미터를 튜닝할 땐
# detection_trigger가 찍힌 시각 대비 absolute_pose(또는 servo_status의
# dwelling=false 전환 시각)가 얼마나 지나 찍히는지를 비교하면 된다.
set -e
cd "$(dirname "$0")"
source install/setup.bash

NAME="${1:-$(date +%Y%m%d_%H%M%S)}"
OUT="bags/${NAME}"
mkdir -p bags

echo "[record.sh] 녹화 시작 -> ${OUT} (Ctrl+C로 종료)"
exec ros2 bag record -o "${OUT}" \
  /arda/tracker/detection_trigger \
  /arda/tracker/absolute_pose \
  /arda/tracker/thermal_image \
  /arda/tracker/detection_confirmed \
  /arda/tracker/radar_frame \
  /arda/tracker/servo_status \
  /arda/tracker/thermal_status \
  /rosout
