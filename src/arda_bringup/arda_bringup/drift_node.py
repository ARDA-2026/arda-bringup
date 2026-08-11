"""drift_node — 표류 예측 및 수색 좌표 도출.

arda-algo_general/hanriver.py (drone_control 브랜치, 드론 미션 연동 버전)의
파티클 필터 표류 시뮬레이션을 ROS2 메시지 기반으로 이식한 노드다.

좌표 규약
---------
`/arda/tracker/absolute_pose` 등에서 쓰는 좌표는 (경도, 위도) WGS84 값이다
(Pose.position.x = 경도, position.y = 위도).

입수 지점 vs 이륙 지점
----------------------
- **입수 지점**(entry): 열화상 카메라가 감지한, 사람이 빠진 자리. 최초
  감지(`/arda/tracker/absolute_pose`) 또는 왼쪽 클릭 관측값
  (`/arda/drift/observation`)으로 처음 설정되면 그 뒤로는 고정된다. 지도
  격자도 이 지점을 기준으로 한 번만 만들어진다.
- **이륙 지점**(takeoff): 드론이 실제로 뜨는 자리 (Tello 기체 좌표계의
  원점). 입수 지점과는 별개 개념이라 `/arda/drift/takeoff` 토픽으로 독립
  적으로 설정한다 — 기본값은 지도 동쪽 바깥 마진에 자동 배치된다.

파티클 필터
-----------
- 왼쪽 클릭(관측값)이나 절대좌표 재수신은 파티클을 재수렴시킨다. 아직
  낙하 지점이 한 번도 설정되지 않았어도(= tracker_node 감지 전) 첫
  관측값을 낙하 지점으로 삼아 필터를 새로 시작한다.
- 유속은 고정값(velocity_x/velocity_y) 또는 HRFCO 실시간 API 중 선택한다.
- 한강 폴리곤(`data/hangang.geojson`)으로 매 스텝 육지 도달을 판정한다.
  강 가운데 섬(폴리곤 내부 구멍)에 부딪히면 반사시켜 비켜 흐르게 하고,
  폴리곤 바깥 테두리를 완전히 벗어나면(진짜 강기슭 도달) "육지 도달"로
  기록한다 — hanriver.py 원본에는 없는 충돌 처리이며, 새로 추가한 부분.

지도/히트맵 (드론 미션 연동을 위한 실물 축소 지도)
--------------------------------------------------
- 입수 지점을 기준으로 고정된 격자를 한 번 만들고(`map_scale`,
  `map_print_w_m/h_m`, `map_east_m`), 매 스텝 지수 감쇠(`hist_half_life_sec`)
  누적 히스토그램으로 갱신한다 — 이전 버전처럼 파티클 평균 위치로 매 프레임
  재중심을 잡지 않는다(격자가 지도 위에 고정돼 있어야 드론 좌표 변환이
  일관된다).
- 확률 상위 Waypoint는 서로 가까운 것끼리 묶이지 않도록 최소 거리
  (`nms_m` = map 폭/4.5) 이상 떨어진 것만 고른다(NMS, `select_spaced`).
- 결과(Waypoint 목록 + 지도/이륙 지점 설정)는 `/arda/drift/waypoints_json`
  (std_msgs/String, JSON)으로 발행한다. 커스텀 ROS 메시지 인터페이스
  패키지를 새로 만들지 않으려고 JSON 문자열로 보내는 실용적 선택이다 —
  자세한 이유는 add.md 참고.
"""
import json
import math

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import (
    Pose,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
    PointStamped,
)

try:
    import requests
except ImportError:  # pragma: no cover - requests가 없으면 API 모드만 비활성화
    requests = None

try:
    from shapely import contains_xy
    from shapely.geometry import shape
    from shapely.ops import unary_union
except ImportError:  # pragma: no cover - shapely가 없으면 육지 감지만 비활성화
    contains_xy = shape = unary_union = None


# 마포대교 인근(위도 약 37.5도) 기준 위경도-미터 환산 근사값.
# hanriver.py / tello_mission.py 와 동일한 상수를 사용한다.
METERS_PER_DEG_LON = 88000.0
METERS_PER_DEG_LAT = 111000.0

GRID_MAX = 80          # 격자 한 축 최대 칸 수 (hanriver.py 동일)
MAX_STRANDED_KEEP = 2000  # 내부 보관 상한 (원본은 무제한 — 메모리 보호용으로 추가)
MAX_STRANDED_PUBLISH = 800  # 매 발행마다 보내는 개수 상한 (원본 stranded_lons[-800:] 동일)


def select_spaced(sorted_wps, min_dist_m, max_count):
    """확률 내림차순 waypoint 중 서로 min_dist_m 이상 떨어진 것만 고른다
    (비최대 억제/NMS). hanriver.py select_spaced() 그대로 이식."""
    picked = []
    for wp in sorted_wps:
        too_close = False
        for p in picked:
            de = (wp['lon'] - p['lon']) * METERS_PER_DEG_LON
            dn = (wp['lat'] - p['lat']) * METERS_PER_DEG_LAT
            if de * de + dn * dn < min_dist_m * min_dist_m:
                too_close = True
                break
        if not too_close:
            picked.append(wp)
            if len(picked) >= max_count:
                break
    return picked


class DriftNode(Node):
    def __init__(self):
        super().__init__('drift_node')

        # --- 파티클 필터 파라미터 (hanriver.py 의 N/TURBULENCE/DIFFUSIVITY/DT/SPEED) ---
        self.declare_parameter('num_particles', 200)
        self.declare_parameter('turbulence', 0.3)
        self.declare_parameter('diffusivity_m', 2.0)
        self.declare_parameter('dt', 0.1)
        # SPEED: 1프레임당 시뮬레이션 스텝 수. 드론 실기 연동 버전은 1을 쓴다
        # (60이면 약 90배속이라 드론이 이륙하기도 전에 파티클이 지도를 벗어남).
        self.declare_parameter('playback_speed', 1)
        self.declare_parameter('step_period_sec', 0.1)
        self.declare_parameter('initial_spread_m', 10.0)
        self.declare_parameter('reacquire_spread_m', 5.0)
        self.declare_parameter('log_interval_sec', 180.0)
        self.declare_parameter('verbose', True)

        # --- 실물 축소 지도 (드론 미션 좌표 변환의 기준, hanriver.py [3-1]) ---
        self.declare_parameter('grid_cell_m', 15.0)      # 격자 한 칸이 덮는 실제 거리
        self.declare_parameter('map_scale', 150.0)        # 축척 1:150
        self.declare_parameter('map_print_w_m', 3.0)      # 실물 지도 가로(m)
        self.declare_parameter('map_print_h_m', 2.0)      # 실물 지도 세로(m)
        self.declare_parameter('map_east_m', 50.0)        # 입수 지점 동쪽 여유(실제 m)
        self.declare_parameter('hist_half_life_sec', 20.0)  # 누적 히트맵 반감기
        self.declare_parameter('nms_max_count', 10)         # NMS 후 최대 waypoint 수

        # --- 드론 이륙 지점 (hanriver.py [3-2]) ---
        # 입수 지점과 분리된 이유: 입수 지점은 감지마다 달라지지만 이륙 자리는
        # 물리적으로 고정돼야 하고, 겹치면 WP1이 이륙 지점과 붙어 Tello 최소
        # 이동거리(20cm) 미만이 되어 스킵된다.
        self.declare_parameter('takeoff_margin_map_m', 0.30)  # 지도 밖으로 나갈 거리(실물 m)

        # 한강 폴리곤(육지 도달 감지). 비워두면 패키지에 내장된
        # data/hangang.geojson (마포대교 구간)을 사용한다.
        self.declare_parameter('enable_land_detection', True)
        self.declare_parameter('river_geojson_path', '')
        self.declare_parameter('river_simplify_deg', 0.00003)  # 폴리곤 단순화 허용 오차 (약 3m)

        # ════════════════════════════════════════════════════════════════
        # 유속(velocity) 설정 — arda-algo_general/hanriver.py 와 동일한 전환 방식
        #
        #   [기본값] 고정 유속 — API 키 없이 바로 실행 가능
        #     velocity_x / velocity_y 파라미터 값을 그대로 사용합니다.
        #     이 값은 아래 launch/bringup.launch.py 의 drift_node 파라미터
        #     블록에서 바꾸거나, 실행 시 다음처럼 직접 넣을 수 있습니다:
        #       ros2 run arda_bringup drift_node --ros-args \
        #         -p velocity_x:=-1.5 -p velocity_y:=0.05
        #
        #   [전환] 한강홍수통제소(HRFCO) 실시간 유속 API 로 바꾸려면:
        #     1) hrfco_api_key 파라미터에 발급받은 API 키를 입력
        #        (발급: https://www.hrfco.go.kr/web/openapiPage/openApi.do)
        #     2) use_hrfco_api 파라미터를 true 로 설정
        #     → velocity_refresh_period_sec 마다 API를 호출해 velocity_x 를
        #       자동 갱신합니다 (hanriver.py 의 get_velocity() 이식).
        #       API 호출 실패 시 hanriver.py와 동일하게 기존 값을 그대로 유지합니다.
        # ════════════════════════════════════════════════════════════════
        self.declare_parameter('velocity_x', -1.5)   # m/s, 서쪽 방향 (한강 평균) — 고정 유속 기본값
        self.declare_parameter('velocity_y', 0.05)   # m/s, 남쪽 방향 — 고정 유속 기본값
        self.declare_parameter('use_hrfco_api', False)   # ← true 로 바꾸면 API 유속 사용
        self.declare_parameter('hrfco_api_key', '')      # ← 여기에 발급받은 HRFCO API 키 입력
        self.declare_parameter('hrfco_obs_code', '1018683')  # 관측소 코드 (한강대교)
        self.declare_parameter('river_width_m', 900.0)       # 유량→유속 역산용 단면 폭 (마포대교 기준)
        self.declare_parameter('river_depth_m', 6.0)         # 유량→유속 역산용 단면 수심 (마포대교 기준)
        self.declare_parameter('velocity_refresh_period_sec', 600.0)  # API 재호출 주기(초)

        self._n = int(self.get_parameter('num_particles').value)
        self._turbulence = float(self.get_parameter('turbulence').value)
        self._diffusivity_deg = (
            float(self.get_parameter('diffusivity_m').value)
            / METERS_PER_DEG_LON * math.sqrt(2 * float(self.get_parameter('dt').value))
        )
        self._dt = float(self.get_parameter('dt').value)
        self._speed = int(self.get_parameter('playback_speed').value)
        self._initial_spread_deg = float(self.get_parameter('initial_spread_m').value) / METERS_PER_DEG_LAT
        self._reacquire_spread_deg = float(self.get_parameter('reacquire_spread_m').value) / METERS_PER_DEG_LAT
        self._log_interval = float(self.get_parameter('log_interval_sec').value)
        self._verbose = bool(self.get_parameter('verbose').value)

        self._grid_cell_m = float(self.get_parameter('grid_cell_m').value)
        self._hist_half_life_sec = float(self.get_parameter('hist_half_life_sec').value)
        self._nms_max_count = int(self.get_parameter('nms_max_count').value)
        self._takeoff_margin_map_m = float(self.get_parameter('takeoff_margin_map_m').value)
        self._hist_decay = 0.5 ** ((self._dt * self._speed) / self._hist_half_life_sec)

        # 지도 설정값은 파라미터 초기값으로 시작하되, POST /map 재구성 시
        # (ROS 파라미터를 매번 다시 쓰는 대신) 이 인스턴스 변수를 직접 갱신한다.
        self._map_scale = float(self.get_parameter('map_scale').value)
        self._map_print_w = float(self.get_parameter('map_print_w_m').value)
        self._map_print_h = float(self.get_parameter('map_print_h_m').value)
        self._map_east_m = float(self.get_parameter('map_east_m').value)

        self._velocity_x = float(self.get_parameter('velocity_x').value)
        self._velocity_y = float(self.get_parameter('velocity_y').value)
        self._use_api = bool(self.get_parameter('use_hrfco_api').value)
        self._api_key = self.get_parameter('hrfco_api_key').value
        self._obs_code = self.get_parameter('hrfco_obs_code').value
        self._river_width = float(self.get_parameter('river_width_m').value)
        self._river_depth = float(self.get_parameter('river_depth_m').value)

        # --- 파티클 필터 상태 ---
        self._initialized = False
        self._entry_lon = None
        self._entry_lat = None
        self._lon = np.zeros(self._n)
        self._lat = np.zeros(self._n)
        self._vlon = np.zeros(self._n)
        self._vlat = np.zeros(self._n)
        self._elapsed_sec = 0.0
        self._last_logged_sec = -self._log_interval

        # --- 지도/격자 상태 (첫 입수 지점 확정 시 _rebuild_map() 이 채움) ---
        self._map_w_m = self._map_h_m = 0.0
        self._map_lon_min = self._map_lon_max = 0.0
        self._map_lat_min = self._map_lat_max = 0.0
        self._grid_nx = self._grid_ny = 0
        self._grid_xedges = self._grid_yedges = None
        self._grid_xcent = self._grid_ycent = None
        self._nms_min_dist_m = 100.0
        self._accumulated_hist = np.zeros((1, 1))
        self._best_lon = self._best_lat = None

        # --- 드론 이륙 지점 ---
        self._takeoff_lon = 0.0
        self._takeoff_lat = 0.0
        self._takeoff_is_default = True

        # --- 육지 도달 감지 상태 (hanriver.py stranded_lons/lats 이식) ---
        self._river_polygon = None
        self._exterior_polygon = None
        self._in_river_prev = None
        self._stranded_lon = []
        self._stranded_lat = []
        if bool(self.get_parameter('enable_land_detection').value):
            self._load_river_polygon()

        self.create_subscription(
            PoseWithCovarianceStamped, '/arda/tracker/absolute_pose',
            self._on_origin, 10)
        self.create_subscription(
            PointStamped, '/arda/drift/observation',
            self._on_observation, 10)
        self.create_subscription(
            PointStamped, '/arda/drift/takeoff',
            self._on_takeoff, 10)
        self.create_subscription(
            Empty, '/arda/drift/takeoff_reset',
            self._on_takeoff_reset, 10)
        self.create_subscription(
            String, '/arda/drift/map_config',
            self._on_map_config, 10)

        self._pub_particles = self.create_publisher(
            PoseArray, '/arda/drift/particles', 10)
        self._pub_target_waypoint = self.create_publisher(
            PoseStamped, '/arda/drift/target_waypoint', 10)
        self._pub_stranded = self.create_publisher(
            PoseArray, '/arda/drift/stranded', 10)
        self._pub_waypoints_json = self.create_publisher(
            String, '/arda/drift/waypoints_json', 10)

        self.create_timer(
            float(self.get_parameter('step_period_sec').value),
            self._on_playback_step)

        if self._use_api and requests is not None:
            self._update_velocity_from_api()
            self.create_timer(
                float(self.get_parameter('velocity_refresh_period_sec').value),
                self._update_velocity_from_api)
        elif self._use_api and requests is None:
            self.get_logger().warn(
                'use_hrfco_api=true 이지만 python3-requests 가 없어 고정 유속을 사용합니다.')

        self.get_logger().info('drift_node started')

    # ------------------------------------------------------------------
    # 유속: 고정값 또는 HRFCO 실시간 API (hanriver.py get_velocity() 이식)
    # ------------------------------------------------------------------
    def _update_velocity_from_api(self):
        import xml.etree.ElementTree as ET

        url = f'https://api.hrfco.go.kr/{self._api_key}/waterlevel/list/10M/{self._obs_code}.xml'
        try:
            response = requests.get(url, timeout=10)
            root = ET.fromstring(response.content)
            item = root.find('.//Waterlevel')
            flow_rate = float(item.find('fw').text)
            velocity = flow_rate / (self._river_width * self._river_depth)
            self._velocity_x = -velocity
            if self._verbose:
                self.get_logger().info(f'[HRFCO] 유속 갱신: {self._velocity_x:.4f} m/s')
        except Exception as exc:
            self.get_logger().warn(f'[HRFCO] API 호출 실패, 기존 유속 유지: {exc}')

    # ------------------------------------------------------------------
    # 한강 폴리곤 로드 (hanriver.py OSM 조회를 대체 — 패키지에 내장된
    # data/hangang.geojson 을 노드 기동 시 한 번만 읽는다)
    # ------------------------------------------------------------------
    def _load_river_polygon(self):
        if shape is None or unary_union is None or contains_xy is None:
            self.get_logger().warn(
                'shapely 가 없어 육지 도달 감지를 비활성화합니다 (pip install shapely).')
            return

        path = self.get_parameter('river_geojson_path').value
        if not path:
            path = f'{get_package_share_directory("arda_bringup")}/data/hangang.geojson'

        try:
            with open(path, encoding='utf-8') as f:
                geo = json.load(f)
            geoms = [shape(feat['geometry']) for feat in geo['features']]
            river_polygon = unary_union(geoms)
            # 시뮬레이션 반경(수백m) 대비 폴리곤 정점이 매우 많아 매 스텝 충돌
            # 판정을 하기엔 느리므로 단순화한다 (섬 등 지역 형태는 유지됨).
            simplify_deg = float(self.get_parameter('river_simplify_deg').value)
            if simplify_deg > 0:
                river_polygon = river_polygon.simplify(simplify_deg, preserve_topology=True)
            self._river_polygon = river_polygon
            self._exterior_polygon = self._exterior_only(river_polygon)
            self.get_logger().info(f'한강 폴리곤 로드 완료: {path}')
        except Exception as exc:
            self.get_logger().warn(f'한강 폴리곤 로드 실패, 육지 도달 감지 비활성화: {exc}')
            self._river_polygon = None
            self._exterior_polygon = None

    @staticmethod
    def _exterior_only(geom):
        """섬(내부 구멍) 없이 바깥 테두리만으로 이루어진 폴리곤 — '강 전체를
        벗어났는지'(진짜 강기슭 도달) 판정용. 섬 충돌은 river_polygon(구멍
        포함)과 이 exterior_polygon의 차이로 구분한다."""
        if geom.geom_type == 'Polygon':
            return type(geom)(geom.exterior)
        if geom.geom_type == 'MultiPolygon':
            return unary_union([type(g)(g.exterior) for g in geom.geoms])
        return geom

    def _publish_stranded(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for lon, lat in zip(self._stranded_lon[-MAX_STRANDED_PUBLISH:],
                             self._stranded_lat[-MAX_STRANDED_PUBLISH:]):
            pose = Pose()
            pose.position.x = float(lon)
            pose.position.y = float(lat)
            msg.poses.append(pose)
        self._pub_stranded.publish(msg)

    # ------------------------------------------------------------------
    # 실물 축소 지도 재구성 (hanriver.py _rebuild_map() 이식)
    # ------------------------------------------------------------------
    def _rebuild_map(self, print_w=None, print_h=None, scale=None, east_m=None):
        """지도 설정을 (재)계산한다. 격자는 지도 영역에 고정되므로, 지도가
        바뀌면 누적 히트맵도 함께 리셋해야 한다 (원본과 동일한 이유)."""
        if print_w is not None:
            self._map_print_w = float(print_w)
        if print_h is not None:
            self._map_print_h = float(print_h)
        if scale is not None:
            self._map_scale = float(scale)
        if east_m is not None:
            self._map_east_m = float(east_m)

        self._map_w_m = self._map_print_w * self._map_scale
        self._map_h_m = self._map_print_h * self._map_scale

        self._map_lon_max = self._entry_lon + self._map_east_m / METERS_PER_DEG_LON
        self._map_lon_min = self._map_lon_max - self._map_w_m / METERS_PER_DEG_LON
        self._map_lat_max = self._entry_lat + (self._map_h_m / 2) / METERS_PER_DEG_LAT
        self._map_lat_min = self._entry_lat - (self._map_h_m / 2) / METERS_PER_DEG_LAT

        cell = max(self._grid_cell_m, self._map_w_m / GRID_MAX, self._map_h_m / GRID_MAX)
        self._grid_nx = int(max(5, min(GRID_MAX, round(self._map_w_m / cell))))
        self._grid_ny = int(max(5, min(GRID_MAX, round(self._map_h_m / cell))))
        self._grid_xedges = np.linspace(self._map_lon_min, self._map_lon_max, self._grid_nx + 1)
        self._grid_yedges = np.linspace(self._map_lat_min, self._map_lat_max, self._grid_ny + 1)
        self._grid_xcent = (self._grid_xedges[:-1] + self._grid_xedges[1:]) / 2
        self._grid_ycent = (self._grid_yedges[:-1] + self._grid_yedges[1:]) / 2

        # waypoint 최소 간격은 지도 크기에 비례 (기본 지도(450m)에서 100m)
        self._nms_min_dist_m = self._map_w_m / 4.5

        self._accumulated_hist = np.zeros((self._grid_nx, self._grid_ny))

        if self._takeoff_is_default:
            self._takeoff_lon, self._takeoff_lat = self._default_takeoff()

        if self._verbose:
            self.get_logger().info(
                f'[MAP] {self._map_print_w}x{self._map_print_h}m 1:{self._map_scale:.0f} = '
                f'실제 {self._map_w_m:.0f}x{self._map_h_m:.0f}m, '
                f'격자 {self._grid_nx}x{self._grid_ny}')

    def _default_takeoff(self):
        """지도 동쪽 변 바깥 takeoff_margin_map_m 지점 (hanriver.py default_takeoff())."""
        east = self._map_east_m + self._takeoff_margin_map_m * self._map_scale
        return self._entry_lon + east / METERS_PER_DEG_LON, self._entry_lat

    def _on_takeoff(self, msg: PointStamped):
        self._takeoff_lon = msg.point.x
        self._takeoff_lat = msg.point.y
        self._takeoff_is_default = False
        self.get_logger().info(f'이륙 지점 설정: ({msg.point.x:.6f}, {msg.point.y:.6f})')

    def _on_takeoff_reset(self, _msg: Empty):
        if not self._initialized:
            return
        self._takeoff_lon, self._takeoff_lat = self._default_takeoff()
        self._takeoff_is_default = True
        self.get_logger().info('이륙 지점을 기본값으로 재설정')

    def _on_map_config(self, msg: String):
        # 비행 중 지도 변경 금지 확인은 web_bridge_node(드론 상태를 아는 쪽)가
        # 이 토픽을 발행하기 전에 이미 처리한다 (hanriver.py post_map()과 동일한 위치).
        if not self._initialized:
            self.get_logger().warn('입수 지점이 아직 없어 지도 설정을 적용할 수 없습니다.')
            return
        try:
            cfg = json.loads(msg.data)
            self._rebuild_map(
                print_w=cfg.get('width_m'), print_h=cfg.get('height_m'),
                scale=cfg.get('scale'), east_m=cfg.get('east_m'))
        except Exception as exc:
            self.get_logger().warn(f'지도 설정 적용 실패: {exc}')

    # ------------------------------------------------------------------
    # 파티클 필터 초기화/재수렴 (hanriver.py 파티클 초기화 + observation 처리)
    # ------------------------------------------------------------------
    def _seed_particles(self, center_lon, center_lat, spread_deg):
        angles = np.random.uniform(0, 2 * np.pi, self._n)
        radii = np.random.uniform(0, spread_deg, self._n)
        self._lon = center_lon + radii * np.cos(angles)
        self._lat = center_lat + radii * np.sin(angles)

        vx_std = abs(self._velocity_x) * self._turbulence
        vy_std = abs(self._velocity_x) * self._turbulence
        self._vlon = (self._velocity_x + np.random.normal(0, vx_std, self._n)) / METERS_PER_DEG_LON
        self._vlat = (self._velocity_y + np.random.normal(0, vy_std, self._n)) / METERS_PER_DEG_LAT
        self._in_river_prev = None  # 재시딩 직후 한 스텝은 "육지 도달"로 오판하지 않도록 초기화

    def _ensure_initialized(self, lon, lat):
        """최초 낙하 지점 확정 시 1회: entry 저장 + 지도/격자 생성."""
        self._entry_lon, self._entry_lat = lon, lat
        self._rebuild_map()
        self._best_lon, self._best_lat = lon, lat

    def _on_origin(self, msg: PoseWithCovarianceStamped):
        lon = msg.pose.pose.position.x
        lat = msg.pose.pose.position.y
        cov_lon = msg.pose.covariance[0]
        cov_lat = msg.pose.covariance[7]

        if not self._initialized:
            self._ensure_initialized(lon, lat)
            spread = math.sqrt(cov_lon) if cov_lon > 0 else self._initial_spread_deg
            self._seed_particles(lon, lat, spread)
            self._initialized = True
            self.get_logger().info(f'파티클 필터 초기화: entry=({lon:.6f}, {lat:.6f})')
        else:
            # 재감지(레이더/열화상) 관측값 → 파티클 재수렴 (입수 지점/지도는 안 바뀜)
            spread = math.sqrt(cov_lat) if cov_lat > 0 else self._reacquire_spread_deg
            self._seed_particles(lon, lat, spread)
            if self._verbose:
                self.get_logger().info(f'재감지 관측값으로 파티클 재수렴: ({lon:.6f}, {lat:.6f})')

    def _on_observation(self, msg: PointStamped):
        # 왼쪽 클릭(관측값, web_bridge_node → 여기). tracker_node가 아직
        # 한 번도 감지하지 못해 필터가 초기화 전이어도, 첫 관측값을 낙하
        # 지점 삼아 바로 시작한다 — hanriver.py는 항상 기본 입수 지점에서
        # 시뮬레이션이 돌고 있어 클릭이 언제나 즉시 반영됐던 것과 동일하게.
        if not self._initialized:
            self._ensure_initialized(msg.point.x, msg.point.y)
            self._seed_particles(msg.point.x, msg.point.y, self._initial_spread_deg)
            self._initialized = True
            self.get_logger().info(
                f'관측값으로 파티클 필터 초기화: ({msg.point.x:.6f}, {msg.point.y:.6f})')
            return
        self._seed_particles(msg.point.x, msg.point.y, self._reacquire_spread_deg)
        if self._verbose:
            self.get_logger().info(
                f'수동 관측값으로 파티클 재수렴: ({msg.point.x:.6f}, {msg.point.y:.6f})')

    # ------------------------------------------------------------------
    # 재생 스텝 (hanriver.py simulation_step() 의 SPEED 루프 이식)
    # ------------------------------------------------------------------
    def _on_playback_step(self):
        if not self._initialized:
            return

        max_vlon = abs(self._velocity_x) * 2 / METERS_PER_DEG_LON
        max_vlat = abs(self._velocity_x) * 1 / METERS_PER_DEG_LAT
        lon_lo, lon_hi = sorted((-max_vlon, -max_vlon * 0.05))

        for _ in range(self._speed):
            new_lon = self._lon + self._vlon * self._dt + np.random.normal(0, self._diffusivity_deg, self._n)
            new_lat = self._lat + self._vlat * self._dt + np.random.normal(0, self._diffusivity_deg, self._n)

            # 섬 등 강 가운데 육지 충돌 처리: 새 위치가 "강 전체 테두리"
            # 안쪽인데도 river_polygon(구멍 포함)에서는 빠지면 섬에 부딪힌
            # 것으로 보고 이번 스텝 이동을 취소하고 속도를 반사(bounce)한다.
            # 테두리 자체를 벗어난 경우(진짜 강기슭 도달)는 그대로 통과시켜
            # 아래 stranded 판정으로 넘긴다.
            if self._river_polygon is not None:
                still_in_river = contains_xy(self._river_polygon, new_lon, new_lat)
                still_in_exterior = contains_xy(self._exterior_polygon, new_lon, new_lat)
                hit_island = still_in_exterior & ~still_in_river
                if hit_island.any():
                    new_lon = np.where(hit_island, self._lon, new_lon)
                    new_lat = np.where(hit_island, self._lat, new_lat)
                    # 유속 방향(vlon)은 서쪽으로만 흐르도록 아래서 다시 clip
                    # 되므로 단순히 부호를 뒤집는 반사는 곧바로 원상복구된다.
                    # 대신 진행 속도를 죽이고 좌우(vlat)로 강하게 튕겨내
                    # 다음 스텝들에서 섬을 비켜 흐르도록 한다.
                    self._vlon = np.where(hit_island, self._vlon * 0.2, self._vlon)
                    lateral_kick = np.random.choice([-1.0, 1.0], size=self._n) * max_vlat
                    self._vlat = np.where(hit_island, lateral_kick, self._vlat)

            self._lon = new_lon
            self._lat = new_lat

            self._vlon += np.random.normal(0, abs(self._velocity_x) * 0.05 / METERS_PER_DEG_LON, self._n)
            self._vlat += np.random.normal(0, abs(self._velocity_x) * 0.05 / METERS_PER_DEG_LAT, self._n)

            self._vlon = np.clip(self._vlon, lon_lo, lon_hi)
            self._vlat = np.clip(self._vlat, -max_vlat, max_vlat)

            self._elapsed_sec += self._dt

        # 육지 도달 감지 (hanriver.py filter_in_river 이식) — 강 폴리곤이
        # 로드되지 않았으면 모든 파티클을 "강 안"으로 취급한다.
        if self._river_polygon is not None:
            in_river = contains_xy(self._river_polygon, self._lon, self._lat)
            if self._in_river_prev is not None:
                newly_stranded = self._in_river_prev & ~in_river
                if newly_stranded.any():
                    self._stranded_lon.extend(self._lon[newly_stranded].tolist())
                    self._stranded_lat.extend(self._lat[newly_stranded].tolist())
                    self._stranded_lon = self._stranded_lon[-MAX_STRANDED_KEEP:]
                    self._stranded_lat = self._stranded_lat[-MAX_STRANDED_KEEP:]
                    self._publish_stranded()
            self._in_river_prev = in_river
        else:
            in_river = np.ones(self._n, dtype=bool)

        lons_v = self._lon[in_river]
        lats_v = self._lat[in_river]

        self._publish_particles(lons_v, lats_v)
        self._update_heatmap_and_publish(lons_v, lats_v)

    def _publish_particles(self, lons_v, lats_v):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        for lon, lat in zip(lons_v, lats_v):
            pose = Pose()
            pose.position.x = float(lon)
            pose.position.y = float(lat)
            msg.poses.append(pose)
        self._pub_particles.publish(msg)

    # ------------------------------------------------------------------
    # 고정 격자 + 지수 감쇠 히트맵 + NMS Waypoint (hanriver.py 후반부 이식)
    # ------------------------------------------------------------------
    def _update_heatmap_and_publish(self, lons_v, lats_v):
        if len(lons_v) > 0:
            hist, _, _ = np.histogram2d(
                lons_v, lats_v, bins=[self._grid_xedges, self._grid_yedges])
            self._accumulated_hist *= self._hist_decay
            self._accumulated_hist += hist

        waypoints = []
        heatmap = []
        total = self._accumulated_hist.sum()
        if total > 0:
            hist_prob = self._accumulated_hist / total * 100
            heatmap = hist_prob.T.tolist()

            max_idx = np.unravel_index(hist_prob.argmax(), hist_prob.shape)
            self._best_lon = float(self._grid_xcent[max_idx[0]])
            self._best_lat = float(self._grid_ycent[max_idx[1]])

            raw = [
                {'lon': float(self._grid_xcent[i]), 'lat': float(self._grid_ycent[j]),
                 'prob': float(hist_prob[i, j])}
                for i, j in zip(*np.nonzero(hist_prob))
            ]
            raw.sort(key=lambda w: w['prob'], reverse=True)
            waypoints = select_spaced(raw, self._nms_min_dist_m, self._nms_max_count)

        # 최우선 목표 좌표 발행
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = self._best_lon
        msg.pose.position.y = self._best_lat
        self._pub_target_waypoint.publish(msg)

        # Waypoint 목록 + 지도/이륙 지점 설정을 JSON으로 발행
        # (드론 미션 계산 · 웹 표시 양쪽에서 이 값을 그대로 신뢰해 씀)
        payload = {
            'elapsed_sec': self._elapsed_sec,
            'in_river_count': int(len(lons_v)),
            'n_particles': self._n,
            'velocity_x': self._velocity_x,
            'waypoints': waypoints[:10],
            'heatmap': heatmap,
            'heatmap_extent': [self._map_lon_min, self._map_lon_max,
                                self._map_lat_min, self._map_lat_max],
            'stranded_count': len(self._stranded_lon),
            'map': {
                'scale': self._map_scale,
                'width_m': self._map_print_w,
                'height_m': self._map_print_h,
                'real_w_m': self._map_w_m,
                'real_h_m': self._map_h_m,
                'east_m': self._map_east_m,
                'grid_nx': self._grid_nx, 'grid_ny': self._grid_ny,
                'nms_m': round(self._nms_min_dist_m, 1),
                'lon_min': self._map_lon_min, 'lon_max': self._map_lon_max,
                'lat_min': self._map_lat_min, 'lat_max': self._map_lat_max,
                'origin_lon': self._entry_lon, 'origin_lat': self._entry_lat,
                'takeoff_lon': self._takeoff_lon, 'takeoff_lat': self._takeoff_lat,
            },
        }
        json_msg = String()
        json_msg.data = json.dumps(payload)
        self._pub_waypoints_json.publish(json_msg)

        if self._verbose and self._elapsed_sec - self._last_logged_sec >= self._log_interval:
            self._last_logged_sec = self._elapsed_sec
            mins, secs = divmod(int(self._elapsed_sec), 60)
            self.get_logger().info(
                f'[T+{mins}m{secs:02d}s] target_waypoint=({self._best_lat:.5f}, {self._best_lon:.5f})')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DriftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
