# 센서(tracker_node, raset을 스레드로 구동) + 알고리즘(drift_node, 표류
# 예측/드론 미션) + 웹 브릿지(web_bridge_node, FastAPI+드론 제어)를 노드
# 3개로 묶은 단일 통합 런치 파일 — 세 패키지를 따로 실행할 필요 없이
# `ros2 launch arda_bringup bringup.launch.py`(또는 워크스페이스 루트의
# `./run.sh`) 한 번으로 전부 뜬다. 빌드도 colcon 한 번(README "🚀 빠른
# 시작" 참고)이면 되고, arda-radar/arda-servo/thermal-camera 세 저장소는
# 코드를 따로 실행하는 게 아니라 tracker_node가 sys.path로 라이브러리처럼
# import만 한다(radar_dir/servo_dir/thermal_dir 파라미터로 경로만 알려줌).
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arda_bringup',
            executable='tracker_node',
            name='tracker_node',
            output='screen',
            parameters=[{
                # ════════════════════════════════════════════════════════
                # 레이더/서보/열화상 원본 저장소 경로 — 반드시 채우세요.
                # (raset 자체는 arda_bringup/raset/ 안에 이미 포함돼 있음.
                #  이 세 저장소만 원본이 이 워크스페이스에 없어서 외부 경로가
                #  필요함 — 실제 로봇/Jetson에 clone해두고 그 경로를 입력)
                #   예: '/home/user/arda-radar'
                # 셋 다 비워두면 tracker_node는 센서 없이 대기 상태로만 남습니다.
                # ════════════════════════════════════════════════════════
                'radar_dir': '/home/orin/ARDA-2026/arda-radar',
                'servo_dir': '/home/orin/ARDA-2026/arda-servo',
                'thermal_dir': '/home/orin/ARDA-2026/thermal-camera',

                # arda-raset main.py의 CLI 옵션과 동일 (필요시만 조정):
                'simulate_servo': False,
                'simulate_thermal': False,
                'no_radar': False,
                # --yolo 플래그와 동일 — true면 열화상 판정을 threshold
                # (원형도/종횡비 기반) 대신 커스텀 YOLO 모델로 한다.
                
                # GPU 연결이 안 됐을 경우, False가 되어 자동으로 CPU로 대체된다(에러는 안 나지만
                # 추론이 느려짐 — thermal_main_yolo.load_yolo_model() 참고).
                'yolo': True,
                'model_path': '',
                'confidence_threshold': 0.4,
                'device': 'cuda',
                # dwell_seconds: 레이더 트리거 후 열화상이 "사람인지" 관찰하는
                # 최대 시간(초). 실기 테스트 때 트리거 후 카메라 앞으로
                # 이동해서 사람 확정(person=True)까지 재현해보려면 기본
                # 10초는 너무 짧다 — 30초로 늘려서 여유를 준다. 다 튜닝되면
                # 운영 환경에선 다시 줄여도 된다(응답성 vs 테스트 편의 트레이드오프).
                'dwell_seconds': 30.0,
                'required_consecutive': 3,
                'settle_offset': 0.15,
                # -1 = 자동으로 dwell_seconds + 30.0 사용 (tracker_node.py
                # _start_raset() 참고). 이 값이 dwell_seconds보다 여유
                # 있게 크지 않으면 레이더가 열화상 판정을 먼저 포기해버려
                # 낙하 확정이 drift_node로 전달 안 되는 버그가 있었음(실측
                # 확인 — 열원이 화면 가장자리에서 계속 "움직이는" 채로
                # 오래 끄는 경우 40초 넘게 걸리는 것도 실측됨, 완전한 상한
                # 보장은 아니니 필요하면 더 늘릴 것) — 반드시 dwell_seconds
                # 보다 충분히 크게 명시하거나 -1(자동)로 둘 것.
                # dwell_seconds를 늘리면 이 값도 자동으로 같이 늘어난다
                # (지금 20+30=50초).
                'thermal_pending_timeout': -1.0,
                'radar_cli_port': '/dev/ttyUSB0',
                'radar_data_port': '/dev/ttyUSB1',

                # 센서 시각화 on/off — 웹 패널의 체크박스로도 별도 제어되며,
                # 여기서 끄면 해당 토픽 자체를 발행하지 않는다.
                'enable_thermal_view': True,
                'enable_radar_view': True,
               
                # raset main.py의 --show-thermal과 동일 — 관찰 중 열화상
                # 컬러맵을 이 보드에 붙은 모니터(X11, DISPLAY 필요)에도
                # 로컬 창으로 띄운다. enable_thermal_view(ROS/웹 발행)와는
                # 독립적 — DISPLAY 없으면 자동으로 무시된다.
                'show_thermal': False,
            }],
        ),
        Node(
            package='arda_bringup',
            executable='drift_node',
            name='drift_node',
            output='screen',
            parameters=[{
                'num_particles': 200,
                'turbulence': 0.3,
                'diffusivity_m': 2.0,
                'dt': 0.1,
                # 1 스텝 tick당 시뮬레이션 스텝 수 (배속). 드론 실기 연동에는 1을
                # 쓴다 — 60이면 약 90배속이라 드론이 이륙하기도 전에 파티클이
                # 지도를 벗어난다. 드론 없이 빠르게 보고 싶으면 값을 올리면 된다.
                'playback_speed': 1,
                'step_period_sec': 0.1,

                # ── 실물 축소 지도 (드론 미션 좌표 변환의 기준) ──
                'map_scale': 150.0,        # 축척 1:150
                'map_print_w_m': 3.0,      # 실물 지도 가로(m)
                'map_print_h_m': 2.0,      # 실물 지도 세로(m)
                'map_east_m': 50.0,        # 입수 지점에서 동쪽 여유(실제 m)
                'grid_cell_m': 15.0,
                'hist_half_life_sec': 20.0,
                'nms_max_count': 10,
                'takeoff_margin_map_m': 0.30,  # 드론 이륙 지점: 지도 밖 여유(실물 m)

                # ════════════════════════════════════════════════════════
                # 유속 설정 — arda-algo_general/hanriver.py 와 동일한 전환 방식
                # (자세한 설명은 drift_node.py 상단 주석 참고)
                # ════════════════════════════════════════════════════════
                # [기본] 고정 유속 — API 키 없이 바로 실행 가능. 값만 바꿔서 사용:
                'velocity_x': -1.5,   # m/s, 서쪽 방향 (한강 평균)
                'velocity_y': 0.05,   # m/s, 남쪽 방향

                # [전환] 실시간 HRFCO API 유속을 쓰려면 아래 두 줄을 채우고
                #        use_hrfco_api 를 True 로 바꾸세요:
                'use_hrfco_api': False,
                'hrfco_api_key': '',       # ← 여기에 HRFCO API 키 입력 (발급: hrfco.go.kr/web/openapiPage/openApi.do)
                'hrfco_obs_code': '1018683',
                'river_width_m': 900.0,
                'river_depth_m': 6.0,
            }],
        ),
        Node(
            package='arda_bringup',
            executable='web_bridge_node',
            name='web_bridge_node',
            output='screen',
            parameters=[{
                # FastAPI 서버 바인드 주소 — 브라우저에서 http://<host>:<port> 로 접속
                'web_server_host': '0.0.0.0',
                'web_server_port': 8000,
            }],
        ),
    ])
