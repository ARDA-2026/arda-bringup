"""tracker_node — 센서 감지 & 융합 모듈.

레이더(arda-radar) + 서보(arda-servo) + 열화상(thermal-camera) 세 프로젝트를
하나의 프로세스 안에서 통합 실행하던 **arda-raset**의 자체 코드
(`raset/bus.py`, `radar_worker.py`, `servo_worker.py`, `thermal_worker.py`,
`thermal_backend.py`)를 이 패키지 `arda_bringup/raset/`에 그대로 옮겨와
(한 줄도 고치지 않고 vendoring) ROS2 노드로 감쌌다. `raset` 자체는 더 이상
외부 경로에 의존하지 않는다 — `src/` 안에서 관리된다.

다만 `raset`의 각 워커가 실제로 import하는 **레이더/서보/열화상 로직
자체**(`arda`, `arda_servo`, `thermal_main` 패키지)는 arda-radar/arda-servo/
thermal-camera 세 저장소 안에 있고, 이 저장소들은 이 워크스페이스 어디에도
없어서(원본이 통째로 존재한 적이 없음) 함께 vendoring할 수 없었다 — 여전히
`radar_dir`/`servo_dir`/`thermal_dir` 파라미터나 `ARDA_RADAR_DIR`/
`ARDA_SERVO_DIR`/`ARDA_THERMAL_DIR` 환경변수로 외부 경로를 알려줘야 한다
(실제 로봇/Jetson에 배포할 때 그 세 저장소를 그 보드에 clone해두고 경로만
알려주면 된다).

⚠️ 이 노드는 이 개발 환경에서 실행/검증할 수 없었다:
  - 위에서 설명한 대로 arda-radar/arda-servo/thermal-camera 세 저장소가
    이 워크스페이스에 없다.
  - 그 세 저장소의 하드웨어 의존성(Jetson.GPIO, Adafruit-Blinka,
    adafruit-circuitpython-mlx90640)은 NVIDIA Jetson(aarch64) 보드 전용이라
    이 x86 dev 환경에는 설치조차 안 된다.
  - 실제 레이더/서보/열화상 하드웨어가 필요하다.
  문법 검사(py_compile)만 통과했습니다 — import/실행 테스트는 못 했으니
  실제 Jetson 환경에서 꼭 직접 확인해 주세요.

토픽 매핑 (raset의 세 이벤트 채널 → ROS)
------------------------------------------------
- `/arda/tracker/detection_trigger` (Bool): 레이더가 낙하 후보를 발견해
  열화상에 확인을 요청하는 순간(raset의 `bus.trigger_q`) — 아직 확정이
  아니라 "이 자리를 봐달라"는 요청 신호다.
- `/arda/tracker/absolute_pose` (PoseWithCovarianceStamped): 열화상이
  "사람 맞음"으로 확정한 순간(raset의 `bus.verdict_q`에서 person=True)의
  GPS 좌표 (position.x=경도, position.y=위도 — 이 패키지 전체의 좌표 규약).
  레이더 단독 후보(열화상 미확인)는 발행하지 않는다 — arda-raset 자체가
  그렇게 설계돼 있다(열화상이 없으면 이 확정 이벤트 자체가 없고, raset은
  로그만 남긴다. `raset/radar_worker.py`의 `thermal_gate=False` 분기 참고).
- `/arda/tracker/thermal_image` (sensor_msgs/Image): 열화상이 낙하 후보를
  관찰(dwell)하는 동안의 프레임(검출 오버레이 포함). raset은 이 프레임을
  기본적으로 `report_url`(HTTP, site 설정이 있을 때만) 또는
  `--show-thermal`(로컬 GUI 창, DISPLAY 필요할 때만)로만 내보내는데, 둘 다
  이 환경/구성에 의존적이라 여기서는 `show=True`로 항상 켜고 `cv2.imshow`를
  ROS 발행으로 갈아끼워서(raset 전체에서 이 함수가 쓰이는 곳은 그 한 줄
  뿐이라 안전하게 가로챌 수 있음) 창을 띄우는 대신 토픽으로 보낸다.

arda-raset 자체를 손대지 않기 위해, bus 큐를 subclass하거나 대체하지 않고
"put()은 원본 큐에 그대로 전달 + 콜백도 같이 호출"하는 극소 Tee 래퍼로
감싼다 — raset의 세 워커 스레드는 원래 큐와 100% 동일하게 동작한다
(get()으로 값을 훔쳐가지 않는다).
"""
import os
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Image

try:
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - 열화상 발행 없이도 나머지는 동작
    CvBridge = None


class _TeeQueue:
    """raset의 bus 큐 하나를 감싸 put() 시점마다 콜백도 같이 호출한다.
    get()은 원본 큐에 그대로 위임하므로 raset 워커의 소비 동작은 전혀
    바뀌지 않는다 — ROS 발행은 부수 효과일 뿐, 큐 값을 가로채 훔치지
    않는다."""

    def __init__(self, inner, on_put):
        self._inner = inner
        self._on_put = on_put

    def put(self, item):
        self._inner.put(item)
        try:
            self._on_put(item)
        except Exception:
            pass  # 콜백 실패가 raset 파이프라인 자체를 죽이면 안 됨

    def get(self, timeout=None):
        return self._inner.get(timeout=timeout)


class TrackerNode(Node):
    def __init__(self):
        super().__init__('tracker_node')

        # ════════════════════════════════════════════════════════════════
        # 레이더/서보/열화상 세 저장소(arda-radar/arda-servo/thermal-camera)
        # 경로. raset 자체는 이 패키지 안에 옮겨왔지만(arda_bringup/raset/),
        # 이 세 저장소는 원본이 이 워크스페이스에 존재한 적이 없어 vendoring
        # 할 수 없었다 — 실제 로봇/Jetson에 clone해두고 아래를 채우세요.
        # 셋 다 비어 있으면(파라미터·환경변수 전부) 이 노드는 센서 없이
        # 대기 상태로만 남는다.
        # ════════════════════════════════════════════════════════════════
        self.declare_parameter('radar_dir', '')        # ← arda-radar 경로. 비우면 ARDA_RADAR_DIR 환경변수
        self.declare_parameter('servo_dir', '')         # ← arda-servo 경로. 비우면 ARDA_SERVO_DIR 환경변수
        self.declare_parameter('thermal_dir', '')        # ← thermal-camera 경로. 비우면 ARDA_THERMAL_DIR 환경변수

        # arda-raset main.py의 CLI 옵션과 1:1 대응하는 파라미터
        self.declare_parameter('simulate_servo', False)
        self.declare_parameter('simulate_thermal', False)
        self.declare_parameter('no_radar', False)
        self.declare_parameter('yolo', False)
        self.declare_parameter('model_path', '')          # 비우면 <thermal_dir>/models/t1_ver3.pt
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('dwell_seconds', 10.0)
        self.declare_parameter('required_consecutive', 3)
        self.declare_parameter('settle_offset', 0.15)
        self.declare_parameter('thermal_pending_timeout', 10.0)
        self.declare_parameter('radar_cli_port', '/dev/ttyUSB0')
        self.declare_parameter('radar_data_port', '/dev/ttyUSB1')
        self.declare_parameter('radar_settings', '')      # 비우면 <radar_dir>/config/settings.yaml
        self.declare_parameter('radar_profile', '')       # 비우면 <radar_dir>/config/profiles/xwr68xx_AOP_profile_short_range.cfg
        self.declare_parameter('servo_config', '')        # 비우면 <servo_dir>/config/settings.yaml

        self._pub_trigger = self.create_publisher(
            Bool, '/arda/tracker/detection_trigger', 10)
        self._pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/arda/tracker/absolute_pose', 10)
        self._pub_thermal_image = self.create_publisher(
            Image, '/arda/tracker/thermal_image', 10)
        # 사람 확정 순간의 열화상 스냅샷 — 연속 스트림(thermal_image)과 별개로,
        # "그 확정 프레임"만 웹 패널 맨 위에 고정해서 보여주기 위한 토픽.
        self._pub_detection_confirmed = self.create_publisher(
            Image, '/arda/tracker/detection_confirmed', 10)

        self._cv_bridge = CvBridge() if CvBridge is not None else None
        self._last_thermal_frame = None  # _on_thermal_frame 이 매 프레임 갱신 (cv2 BGR ndarray)
        self._stop_event = threading.Event()
        self._threads: list = []

        try:
            self._start_raset()
        except Exception as exc:
            self.get_logger().error(
                f'센서 서브시스템 시작 실패 — 센서 융합 없이 대기 상태로 남습니다: {exc}')

        self.get_logger().info('tracker_node started')

    # ------------------------------------------------------------------
    def _resolve_dir(self, param_name: str, env_name: str) -> Path:
        p = self.get_parameter(param_name).value
        if p:
            return Path(p).resolve()
        if os.environ.get(env_name):
            return Path(os.environ[env_name]).resolve()
        raise RuntimeError(
            f'{param_name} 파라미터(또는 {env_name} 환경변수)가 비어 있습니다')

    def _start_raset(self):
        radar_dir = self._resolve_dir('radar_dir', 'ARDA_RADAR_DIR')
        servo_dir = self._resolve_dir('servo_dir', 'ARDA_SERVO_DIR')
        thermal_dir = self._resolve_dir('thermal_dir', 'ARDA_THERMAL_DIR')
        for d, label in ((radar_dir, 'arda-radar'), (servo_dir, 'arda-servo'), (thermal_dir, 'thermal-camera')):
            if not d.is_dir():
                raise RuntimeError(
                    f'{label} 디렉터리를 찾을 수 없습니다: {d} — '
                    f'radar_dir/servo_dir/thermal_dir 파라미터나 '
                    f'ARDA_RADAR_DIR/ARDA_SERVO_DIR/ARDA_THERMAL_DIR 환경변수로 지정하세요.')

        sys.path.insert(0, str(radar_dir))                 # -> import arda
        sys.path.insert(0, str(servo_dir))                  # -> import arda_servo
        sys.path.insert(0, str(thermal_dir / 'src'))         # -> import thermal_main / thermal_main_yolo

        import yaml
        from arda.utils import load_settings
        from .raset import radar_worker, servo_worker, thermal_worker, thermal_backend
        from .raset.bus import Bus

        # cv2.imshow는 raset 전체에서 thermal_worker.py의 --show-thermal
        # 경로 한 곳에서만 쓰인다 — 로컬 창을 띄우는 대신 ROS로 발행하도록
        # 이 함수만 안전하게 갈아끼운다 (raset 코드 자체는 수정하지 않음).
        import cv2
        cv2.imshow = self._on_thermal_frame
        cv2.waitKey = lambda *a, **k: -1

        radar_settings_param = self.get_parameter('radar_settings').value
        radar_settings_path = (
            Path(radar_settings_param) if radar_settings_param
            else radar_dir / 'config' / 'settings.yaml')
        radar_profile_param = self.get_parameter('radar_profile').value
        radar_profile_path = (
            Path(radar_profile_param) if radar_profile_param
            else radar_dir / 'config' / 'profiles' / 'xwr68xx_AOP_profile_short_range.cfg')
        servo_config_param = self.get_parameter('servo_config').value
        servo_config_path = (
            Path(servo_config_param) if servo_config_param
            else servo_dir / 'config' / 'settings.yaml')

        report_url = load_settings(radar_settings_path).get('site', {}).get('report_url', '')

        with open(servo_config_path, encoding='utf-8') as f:
            servo_cfg = yaml.safe_load(f)

        bus = Bus()
        # detection_trigger 발행: 원본 소비자(thermal_worker)는 그대로 두고
        # put() 시점에만 콜백을 얹는다.
        bus.trigger_q = _TeeQueue(bus.trigger_q, self._on_trigger)
        # absolute_pose 발행: verdict(person 확정 여부)가 나오는 순간
        # pending_location(그 판정을 기다리던 좌표)을 같이 스냅샷한다.
        # radar_worker가 나중에 pending_location.clear()를 하기 전에 이
        # put() 콜백이 (같은 스레드 호출 안에서) 먼저 실행되므로 안전하다.
        bus.verdict_q = _TeeQueue(bus.verdict_q, lambda v: self._on_verdict(v, bus))

        simulate_servo = bool(self.get_parameter('simulate_servo').value)
        simulate_thermal = bool(self.get_parameter('simulate_thermal').value)
        no_radar = bool(self.get_parameter('no_radar').value)
        use_yolo = bool(self.get_parameter('yolo').value)
        model_path_param = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        dwell_seconds = float(self.get_parameter('dwell_seconds').value)
        required_consecutive = int(self.get_parameter('required_consecutive').value)
        settle_offset = float(self.get_parameter('settle_offset').value)
        thermal_pending_timeout = float(self.get_parameter('thermal_pending_timeout').value)
        radar_cli_port = self.get_parameter('radar_cli_port').value
        radar_data_port = self.get_parameter('radar_data_port').value

        def _spawn(name, target, *fn_args):
            def _wrapped():
                try:
                    target(*fn_args)
                except Exception:
                    self.get_logger().error(f'[{name}] 처리되지 않은 예외로 종료됨')
                    self._stop_event.set()
            t = threading.Thread(target=_wrapped, name=name, daemon=True)
            t.start()
            self._threads.append(t)

        # 서보는 항상 기동한다 — GPIO가 없으면 arda-servo가 자동으로
        # 시뮬레이션 모드로 떨어진다 (arda-raset main.py와 동일).
        _spawn('servo', servo_worker.run, bus, self._stop_event, servo_cfg, simulate_servo)

        # 열화상 — 센서 초기화를 스레드를 띄우기 전에 미리 해서, 실패 시
        # (하드웨어 없음, simulate_thermal 아님) 스레드 자체를 안 띄운다.
        thermal_started = False
        try:
            i2c, read_frame_fn = thermal_backend.initialize_sensor(simulate=simulate_thermal)
        except RuntimeError as exc:
            self.get_logger().info(f'열화상 센서를 사용할 수 없어 생략합니다: {exc}')
        else:
            try:
                if use_yolo:
                    model_path = model_path_param or str(thermal_dir / 'models' / 't1_ver3.pt')
                    backend = thermal_backend.YoloBackend(model_path, device, confidence_threshold)
                else:
                    backend = thermal_backend.ThresholdBackend()
            except Exception as exc:
                self.get_logger().error(f'열화상 판정 백엔드 초기화 실패 — 열화상 없이 진행합니다: {exc}')
                if i2c is not None and hasattr(i2c, 'deinit'):
                    i2c.deinit()
            else:
                _spawn(
                    'thermal', thermal_worker.run, bus, self._stop_event, backend, read_frame_fn, i2c,
                    dwell_seconds, required_consecutive, settle_offset, report_url,
                    True,  # show=True — 위에서 cv2.imshow를 ROS 발행으로 갈아끼웠으므로 항상 켠다
                )
                thermal_started = True

        # 레이더 — USB 시리얼 포트가 없으면 자동으로 생략한다.
        if no_radar:
            self.get_logger().info('no_radar 파라미터로 레이더 생략')
        elif not (Path(radar_cli_port).exists() and Path(radar_data_port).exists()):
            self.get_logger().info(
                f'{radar_cli_port}, {radar_data_port} 가 없어 레이더는 생략합니다 (USB 연결 확인)')
        else:
            _spawn(
                'radar', radar_worker.run, bus, self._stop_event,
                radar_cli_port, radar_data_port, radar_profile_path, radar_settings_path,
                thermal_started, thermal_pending_timeout, report_url,
            )

        self.get_logger().info(
            f'arda-raset 서브시스템 시작됨 (thermal={thermal_started}, radar_dir={radar_dir}, servo_dir={servo_dir})')

    # ------------------------------------------------------------------
    # raset bus/cv2 탭 → ROS 발행
    # ------------------------------------------------------------------
    def _on_trigger(self, _ts: float):
        msg = Bool()
        msg.data = True
        self._pub_trigger.publish(msg)

    def _on_verdict(self, verdict, bus):
        if not getattr(verdict, 'person', False):
            return
        loc = bus.pending_location.get()
        if loc is None:
            return
        stamp = self.get_clock().now().to_msg()

        msg = PoseWithCovarianceStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = loc.lon
        msg.pose.pose.position.y = loc.lat
        self._pub_pose.publish(msg)

        # 확정된 그 순간의 열화상 프레임을 스냅샷으로 같이 보낸다 (있으면).
        # thermal_worker의 관찰 루프 안에서 person=True가 나오는 프레임이
        # 곧 cv2.imshow(=_on_thermal_frame)로 마지막에 찍힌 프레임이므로
        # _last_thermal_frame이 바로 그 확정 프레임이다.
        if self._last_thermal_frame is not None and self._cv_bridge is not None:
            try:
                img_msg = self._cv_bridge.cv2_to_imgmsg(self._last_thermal_frame, encoding='bgr8')
                img_msg.header.stamp = stamp
                self._pub_detection_confirmed.publish(img_msg)
            except Exception as exc:
                self.get_logger().warn(f'확정 열화상 스냅샷 발행 실패: {exc}')

    def _on_thermal_frame(self, _window_name, image):
        # cv2.imshow(window_name, image) 대신 호출된다 (thermal_worker.py 참고).
        self._last_thermal_frame = image
        if self._cv_bridge is None:
            return
        try:
            msg = self._cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            self._pub_thermal_image.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f'열화상 프레임 발행 실패: {exc}')

    def destroy_node(self):
        self._stop_event.set()
        for t in self._threads:
            t.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
