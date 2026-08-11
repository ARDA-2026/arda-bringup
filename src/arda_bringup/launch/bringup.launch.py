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
                'radar_dir': '',    # ← arda-radar 경로 입력
                'servo_dir': '',    # ← arda-servo 경로 입력
                'thermal_dir': '',  # ← thermal-camera 경로 입력

                # arda-raset main.py의 CLI 옵션과 동일 (필요시만 조정):
                'simulate_servo': False,
                'simulate_thermal': False,
                'no_radar': False,
                'yolo': False,
                'dwell_seconds': 10.0,
                'required_consecutive': 3,
                'settle_offset': 0.15,
                'thermal_pending_timeout': 10.0,
                'radar_cli_port': '/dev/ttyUSB0',
                'radar_data_port': '/dev/ttyUSB1',
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
