"""web_bridge_node — 웹 관제 브리지 노드.

arda-algo_general/hanriver.py(drone_control 브랜치)는 원래 파티클
시뮬레이션 + FastAPI 웹 서버 + 드론 제어가 한 프로세스에 같이 있었다.
이 노드는 시뮬레이션은 drift_node 로 옮기고, FastAPI + WebSocket + 정적
페이지(static/index.html) + 드론 제어 라우터(drone_api)를 그대로 옮겨와
"ROS 토픽 → 웹/드론" 브리지로 동작하게 한 것이다.

- `/arda/tracker/absolute_pose`, `/arda/drift/particles`,
  `/arda/drift/target_waypoint`, `/arda/drift/stranded`,
  `/arda/drift/waypoints_json`, `/arda/tracker/thermal_image` 를 구독해
  hanriver.py 의 `sim_state` 와 동일한 JSON 스키마를 만들어 `/state`
  (REST) 와 `/ws` (WebSocket, 100ms 주기) 로 내보낸다.
- 히트맵/Waypoint/지도(map) 설정은 drift_node가 `/arda/drift/waypoints_json`
  (JSON 문자열)으로 이미 계산해 보낸 것을 그대로 신뢰해서 쓴다 — 커스텀
  ROS 메시지 인터페이스 패키지를 새로 만들지 않으려는 실용적 선택이다
  (자세한 이유는 add.md 참고). 이전 버전처럼 여기서 다시 계산하지 않는다.
- 브라우저 왼쪽 클릭(POST /observation)은 `/arda/drift/observation` 토픽
  으로 발행해 drift_node 의 파티클 필터를 관측값으로 재수렴시킨다.
- 브라우저에서 "🚁 이륙 지점" 버튼을 누른 뒤 클릭(POST /takeoff)하면
  `/arda/drift/takeoff` 토픽으로 발행해 드론 이륙 지점(Tello 기체 좌표계의
  원점)을 지정한다. 입수 지점과는 별개 개념이라 파티클 필터는 건드리지
  않는다. `/takeoff/reset` 은 기본 위치(지도 동쪽 바깥)로 되돌린다.
- `/map`, `/map/reset` 은 실물 축소 지도(스케일/크기) 설정을 바꾼다.
  `/arda/drift/map_config` 토픽으로 발행하며, 드론이 비행 중일 때는 여기서
  거부한다(드론 상태를 아는 것은 이 노드뿐이라 이 계층에서 막는다).
- `/river-geojson`은 패키지에 내장된 `data/hangang.geojson`(마포대교 구간,
  osmnx로 미리 생성)을 그대로 반환한다.
- `/arda/tracker/detection_confirmed`(tracker_node가 사람 확정 순간에만
  발행하는 열화상 스냅샷)를 구독해 JPEG→base64로 인코딩한 뒤
  `confirmed_thermal_jpg_b64`/`confirmed_at`로 `/state`·`/ws`에 실어
  보낸다 — 연속 스트림(`thermal_image`)과 달리 확정 이벤트 1건마다만
  갱신되는 값이라, 프론트엔드가 패널 맨 위 카드에 "그 순간" 이미지+시간을
  고정해서 보여줄 수 있다.
- 드론 제어(`/drone/*`, `/mission`)는 `drone_api.router` 를 그대로 마운트
  하고, `drone_api.set_state_provider()` 로 이 노드의 `self._state` 를
  주입한다 — hanriver.py 원본이 `drone_api.set_state_provider(_state_snapshot)`
  로 하던 것과 동일한 패턴이다. 실기체 제어는 `tello_driver.py`(dry_run
  기본값 true, 배터리/지도경계 검사, 예외 시 강제착륙 등 안전장치 그대로
  포팅됨)가 담당한다.

`web_server_host` / `web_server_port` 파라미터는 (add.md 에 기록했듯) 원래
"외부 웹 서버로 나가는 커넥션" 용도로 설계됐으나, hanriver.py 코드 자체가
서버이므로 이 노드가 직접 그 주소로 FastAPI 서버를 호스팅하는 용도로
재해석해 사용한다.
"""
import asyncio
import base64
import json
import threading
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import (
    PointStamped,
    PoseArray,
    PoseStamped,
    PoseWithCovarianceStamped,
)
from sensor_msgs.msg import Image

import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse, Response
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, Field

from . import drone_api

try:
    from cv_bridge import CvBridge
    import cv2
except ImportError:  # pragma: no cover - 열화상 스트리밍 없이도 나머지는 동작
    CvBridge = None
    cv2 = None


class ObservationIn(BaseModel):
    lon: float
    lat: float


class MapIn(BaseModel):
    width_m: float = Field(3.0, ge=0.3, le=20.0)
    height_m: float = Field(2.0, ge=0.3, le=20.0)
    scale: float = Field(150.0, ge=10.0, le=2000.0)
    east_m: float = Field(50.0, ge=0.0, le=5000.0)


class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')

        self.declare_parameter('web_server_host', '127.0.0.1')
        self.declare_parameter('web_server_port', 8000)
        self.declare_parameter('velocity_x_display', -1.5)
        self.declare_parameter('max_trail_len', 200)
        self.declare_parameter('max_obs_history', 50)

        self._velocity_x_display = float(self.get_parameter('velocity_x_display').value)
        self._max_trail = int(self.get_parameter('max_trail_len').value)
        self._max_obs = int(self.get_parameter('max_obs_history').value)

        # --- 공유 상태 (ROS 콜백 스레드 ↔ FastAPI 스레드) ---
        # drone_api.set_state_provider() 가 그대로 반환하는 dict이기도 하므로
        # hanriver.py sim_state 와 동일한 키 이름을 유지한다.
        self._lock = threading.Lock()
        self._state: dict = {
            'elapsed_sec': 0.0,
            'particles_lon': [], 'particles_lat': [],
            'heatmap': [], 'heatmap_extent': [0, 0, 0, 0],
            # index.html 이 null 체크 없이 toFixed()/toCanvas() 를 호출하므로
            # 첫 target_waypoint/absolute_pose 수신 전에도 숫자 기본값을 준다.
            'best_lon': 0.0, 'best_lat': 0.0,
            'best_trail_lons': [], 'best_trail_lats': [],
            'waypoints': [],
            'stranded_lons': [], 'stranded_lats': [], 'stranded_count': 0,
            'observation': None, 'obs_history': [],
            'in_river_count': 0, 'n_particles': 0,
            'velocity_x': self._velocity_x_display,
            'entry_lon': 0.0, 'entry_lat': 0.0,
            'map': None,
            # 사람 확정 순간의 열화상 스냅샷 (패널 맨 위 카드용) — 확정 전엔 둘 다 None
            'confirmed_thermal_jpg_b64': None,
            'confirmed_at': None,
        }
        self._start_time = self.get_clock().now()
        self._has_entry = False
        self._latest_jpeg: Optional[bytes] = None
        self._cv_bridge = CvBridge() if CvBridge is not None else None
        self._river_geojson = self._load_river_geojson()

        self._pub_observation = self.create_publisher(
            PointStamped, '/arda/drift/observation', 10)
        self._pub_takeoff = self.create_publisher(
            PointStamped, '/arda/drift/takeoff', 10)
        self._pub_takeoff_reset = self.create_publisher(
            Empty, '/arda/drift/takeoff_reset', 10)
        self._pub_map_config = self.create_publisher(
            String, '/arda/drift/map_config', 10)

        self.create_subscription(
            PoseWithCovarianceStamped, '/arda/tracker/absolute_pose',
            self._on_pose, 10)
        self.create_subscription(
            Image, '/arda/tracker/thermal_image',
            self._on_thermal_image, 10)
        self.create_subscription(
            Image, '/arda/tracker/detection_confirmed',
            self._on_detection_confirmed, 10)
        self.create_subscription(
            PoseArray, '/arda/drift/particles',
            self._on_particles, 10)
        self.create_subscription(
            PoseStamped, '/arda/drift/target_waypoint',
            self._on_target_waypoint, 10)
        self.create_subscription(
            PoseArray, '/arda/drift/stranded',
            self._on_stranded, 10)
        self.create_subscription(
            String, '/arda/drift/waypoints_json',
            self._on_waypoints_json, 10)

        # 드론 제어 라우터에 상태 제공자 등록 (hanriver.py 의
        # drone_api.set_state_provider(_state_snapshot) 와 동일한 패턴)
        drone_api.set_state_provider(self._state_snapshot)

        self._app = self._build_app()
        host = self.get_parameter('web_server_host').value
        port = int(self.get_parameter('web_server_port').value)
        self._server_thread = threading.Thread(
            target=self._run_server, args=(host, port), daemon=True)
        self._server_thread.start()

        self.get_logger().info(f'web_bridge_node started — http://{host}:{port}')

    def _state_snapshot(self) -> dict:
        with self._lock:
            return dict(self._state)

    # ------------------------------------------------------------------
    # FastAPI 앱 (hanriver.py 의 엔드포인트 + drone_api 라우터를 그대로 이식)
    # ------------------------------------------------------------------
    def _build_app(self) -> FastAPI:
        app = FastAPI(title='ARDA Web Bridge')
        static_dir = f'{get_package_share_directory("arda_bringup")}/web/static'
        app.mount('/static', StaticFiles(directory=static_dir), name='static')
        app.include_router(drone_api.router)

        @app.get('/', response_class=HTMLResponse)
        async def index():
            with open(f'{static_dir}/index.html', encoding='utf-8') as f:
                # 캐시 금지 — UI를 고쳐도 브라우저가 옛 파일을 계속 쓰면
                # "고쳤는데 왜 그대로냐"로 시간을 버린다 (hanriver.py와 동일).
                return HTMLResponse(f.read(), headers={
                    'Cache-Control': 'no-store, no-cache, must-revalidate',
                    'Pragma': 'no-cache',
                })

        @app.get('/river-geojson')
        async def river_geojson():
            return self._river_geojson

        @app.get('/state')
        async def get_state():
            return self._state_snapshot()

        @app.get('/thermal.jpg')
        async def thermal_jpg():
            if self._latest_jpeg is None:
                return Response(status_code=404)
            return Response(content=self._latest_jpeg, media_type='image/jpeg')

        @app.post('/observation')
        async def post_observation(obs: ObservationIn):
            self._publish_observation(obs.lon, obs.lat)
            return {'ok': True}

        @app.post('/takeoff')
        async def post_takeoff(pt: ObservationIn):
            """드론 이륙 지점을 지정한다. 비행 중엔 지령 위치 누적이
            어긋나므로 거부한다 (hanriver.py post_takeoff()와 동일)."""
            if drone_api.driver.status()['running']:
                raise HTTPException(409, '비행 중에는 이륙 지점을 바꿀 수 없습니다')
            self._publish_takeoff(pt.lon, pt.lat)
            return {'ok': True, 'lon': pt.lon, 'lat': pt.lat}

        @app.post('/takeoff/reset')
        async def post_takeoff_reset():
            if drone_api.driver.status()['running']:
                raise HTTPException(409, '비행 중에는 이륙 지점을 바꿀 수 없습니다')
            self._pub_takeoff_reset.publish(Empty())
            return {'ok': True}

        @app.post('/map')
        async def post_map(cfg: MapIn):
            if drone_api.driver.status()['running']:
                raise HTTPException(409, '비행 중에는 지도를 바꿀 수 없습니다')
            if cfg.east_m > cfg.width_m * cfg.scale:
                raise HTTPException(
                    422, f'동쪽 여유({cfg.east_m:.0f}m)가 지도 가로'
                         f'({cfg.width_m * cfg.scale:.0f}m)보다 큽니다')
            self._publish_map_config(cfg.width_m, cfg.height_m, cfg.scale, cfg.east_m)
            return {'ok': True, **cfg.model_dump(),
                    'real_w_m': cfg.width_m * cfg.scale,
                    'real_h_m': cfg.height_m * cfg.scale}

        @app.post('/map/reset')
        async def post_map_reset():
            if drone_api.driver.status()['running']:
                raise HTTPException(409, '비행 중에는 지도를 바꿀 수 없습니다')
            self._publish_map_config(3.0, 2.0, 150.0, 50.0)
            return {'ok': True}

        @app.websocket('/ws')
        async def websocket_endpoint(ws: WebSocket):
            await ws.accept()
            try:
                while True:
                    data = json.dumps(self._state_snapshot())
                    await ws.send_text(data)
                    await asyncio.sleep(0.1)
            except WebSocketDisconnect:
                pass

        return app

    def _run_server(self, host: str, port: int):
        config = uvicorn.Config(self._app, host=host, port=port, log_level='warning')
        server = uvicorn.Server(config)
        server.run()

    def _load_river_geojson(self) -> dict:
        path = f'{get_package_share_directory("arda_bringup")}/data/hangang.geojson'
        try:
            with open(path, encoding='utf-8') as f:
                geo = json.load(f)
            self.get_logger().info(f'한강 폴리곤 GeoJSON 로드 완료: {path}')
            return geo
        except Exception as exc:
            self.get_logger().warn(f'한강 폴리곤 GeoJSON 로드 실패: {exc}')
            return {'type': 'FeatureCollection', 'features': []}

    # ------------------------------------------------------------------
    # 브라우저 → ROS
    # ------------------------------------------------------------------
    def _publish_observation(self, lon: float, lat: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = lon
        msg.point.y = lat
        self._pub_observation.publish(msg)

        with self._lock:
            self._state['observation'] = [lon, lat]
            self._state['obs_history'].append([lon, lat])
            self._state['obs_history'] = self._state['obs_history'][-self._max_obs:]

    def _publish_takeoff(self, lon: float, lat: float):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = lon
        msg.point.y = lat
        self._pub_takeoff.publish(msg)

    def _publish_map_config(self, width_m, height_m, scale, east_m):
        msg = String()
        msg.data = json.dumps({
            'width_m': width_m, 'height_m': height_m,
            'scale': scale, 'east_m': east_m,
        })
        self._pub_map_config.publish(msg)
        with self._lock:
            self._state['best_trail_lons'] = []
            self._state['best_trail_lats'] = []

    # ------------------------------------------------------------------
    # ROS → 브라우저
    # ------------------------------------------------------------------
    def _on_pose(self, msg: PoseWithCovarianceStamped):
        lon = msg.pose.pose.position.x
        lat = msg.pose.pose.position.y

        with self._lock:
            if not self._has_entry:
                self._state['entry_lon'] = lon
                self._state['entry_lat'] = lat
                self._has_entry = True
            else:
                # 레이더/열화상 재감지 → 관측값 기록
                self._state['observation'] = [lon, lat]
                self._state['obs_history'].append([lon, lat])
                self._state['obs_history'] = self._state['obs_history'][-self._max_obs:]

    def _on_thermal_image(self, msg: Image):
        if self._cv_bridge is None or cv2 is None:
            return
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', frame)
            if ok:
                self._latest_jpeg = buf.tobytes()
        except Exception as exc:
            self.get_logger().warn(f'열화상 인코딩 실패: {exc}')

    def _on_detection_confirmed(self, msg: Image):
        # 사람 확정 순간의 열화상 스냅샷 — 패널 맨 위 카드에 시간과 함께 표시.
        # base64로 JSON(/state, /ws)에 직접 실어 보내서 별도 폴링/캐시 문제
        # 없이 WebSocket 갱신만으로 바로 뜨게 한다.
        if self._cv_bridge is None or cv2 is None:
            return
        try:
            frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            ok, buf = cv2.imencode('.jpg', frame)
            if not ok:
                return
            b64 = base64.b64encode(buf.tobytes()).decode('ascii')
        except Exception as exc:
            self.get_logger().warn(f'확정 열화상 인코딩 실패: {exc}')
            return

        stamp_sec = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        with self._lock:
            self._state['confirmed_thermal_jpg_b64'] = b64
            self._state['confirmed_at'] = stamp_sec
        self.get_logger().info('사람 확정 열화상 스냅샷 수신 — 패널에 표시')

    def _on_particles(self, msg: PoseArray):
        with self._lock:
            self._state['particles_lon'] = [p.position.x for p in msg.poses]
            self._state['particles_lat'] = [p.position.y for p in msg.poses]
            self._state['elapsed_sec'] = (
                (self.get_clock().now() - self._start_time).nanoseconds / 1e9)

    def _on_stranded(self, msg: PoseArray):
        # drift_node가 계산한 누적 육지 도달 목록을 그대로 반영 (표시용 재계산 아님)
        with self._lock:
            self._state['stranded_lons'] = [p.position.x for p in msg.poses]
            self._state['stranded_lats'] = [p.position.y for p in msg.poses]

    def _on_target_waypoint(self, msg: PoseStamped):
        with self._lock:
            best_lon = msg.pose.position.x
            best_lat = msg.pose.position.y
            self._state['best_lon'] = best_lon
            self._state['best_lat'] = best_lat
            self._state['best_trail_lons'].append(best_lon)
            self._state['best_trail_lats'].append(best_lat)
            self._state['best_trail_lons'] = self._state['best_trail_lons'][-self._max_trail:]
            self._state['best_trail_lats'] = self._state['best_trail_lats'][-self._max_trail:]

    def _on_waypoints_json(self, msg: String):
        # drift_node가 계산한 waypoints/heatmap/지도 설정을 그대로 반영
        # (여기서 다시 계산하지 않음 — drift_node가 유일한 권위있는 소스).
        try:
            payload = json.loads(msg.data)
        except Exception as exc:
            self.get_logger().warn(f'waypoints_json 파싱 실패: {exc}')
            return
        with self._lock:
            self._state['waypoints'] = payload.get('waypoints', [])
            self._state['heatmap'] = payload.get('heatmap', [])
            self._state['heatmap_extent'] = payload.get('heatmap_extent', [0, 0, 0, 0])
            self._state['stranded_count'] = payload.get('stranded_count', 0)
            self._state['in_river_count'] = payload.get('in_river_count', 0)
            self._state['n_particles'] = payload.get('n_particles', 0)
            self._state['velocity_x'] = payload.get('velocity_x', self._velocity_x_display)
            self._state['map'] = payload.get('map')

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WebBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
