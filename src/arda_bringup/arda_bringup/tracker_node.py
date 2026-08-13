"""tracker_node — 센서 감지 & 융합 모듈.

레이더(arda-radar) + 서보(arda-servo) + 열화상(thermal-camera) 세 프로젝트를
하나의 프로세스 안에서 통합 실행하던 **arda-raset**의 자체 코드
(`raset/bus.py`, `radar_worker.py`, `servo_worker.py`, `thermal_worker.py`,
`thermal_backend.py`)를 이 패키지 `arda_bringup/raset/`에 그대로 옮겨와
ROS2 노드로 감쌌다. `raset` 자체는 더 이상 외부 경로에 의존하지 않는다 —
`src/` 안에서 관리된다.

원칙은 "raset 로직은 안 고친다"이지만 딱 두 곳, 웹 시각화를 위해 최소한의
추가(기존 로직 변경 없이 데이터를 한 줄 더 내보내기만 함)를 했다:
`raset/bus.py`에 `radar_frame_q` 필드 하나, `raset/radar_worker.py`
클러스터링 직후에 그 큐로 `.put()` 한 줄. 레이더 포인트클라우드는 raset
어디에도 노출되는 곳이 없어(`RealtimePlotter`는 쓰지 않음, 아래 참고)
이것 없이는 웹에 띄울 방법이 없었다.

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
- `/arda/tracker/thermal_image` (sensor_msgs/Image): 열화상 프레임 — 레이더
  트리거 대기 중(원본 컬러맵)에도, 낙하 후보를 관찰(dwell)하는 동안(검출
  오버레이 포함)에도 상시로 발행된다(raset의 always-on 스트리밍, `show`가
  켜져 있으면 대기 상태에서도 매 프레임 읽어 표시/전송함). raset은 이
  프레임을 기본적으로 `report_url`(HTTP, site 설정이 있을 때만) 또는
  `--show-thermal`(로컬 GUI 창, DISPLAY 필요할 때만)로만 내보내는데, 둘 다
  이 환경/구성에 의존적이라 여기서는 `show=True`로 항상 켜고 `cv2.imshow`를
  ROS 발행으로 갈아끼워서(raset 전체에서 이 함수가 쓰이는 곳은 그 한 줄
  뿐이라 안전하게 가로챌 수 있음) 창을 띄우는 대신 토픽으로 보낸다.
  `enable_thermal_view` 파라미터(기본 true)로 끄면 이 토픽만 발행을
  건너뛴다(센서 읽기 자체는 계속함) — detection_confirmed는 이 스위치와
  무관하게 항상 발행됨.
- `/arda/tracker/radar_frame` (std_msgs/String, JSON): 레이더 포인트클라우드
  시각화 전용 스냅샷(`points`, `n_clusters`, `cluster_centroids`) — 5Hz로
  발행. `enable_radar_view` 파라미터(기본 true)로 끌 수 있다.
- `/arda/tracker/servo_status` (std_msgs/String, JSON): 서보 각도/dwell
  상태 스냅샷(`angle`, `dwelling`, `dwell_remaining_sec`,
  `thermal_engaged`) — 5Hz로 항상 발행(on/off 스위치 없음, 오버헤드
  미미함). rosbag 기록으로 레이더/열화상/서보 타이밍을 함께 분석할 때 씀.
- `/arda/tracker/thermal_status` (std_msgs/String, JSON): 열화상 판정
  진행 상태(`frame_number`, `matched`, `match_count`,
  `required_matches`, `moving`, `confirmed`, `offset`,
  `give_up_remaining_sec`) — 관찰(dwell) 중에만 매 프레임 발행. "사람
  확정까지 얼마나 가까워졌는지"(누적 매칭 횟수 등, 연속일 필요 없음)를
  dwell_seconds 튜닝하면서 바로 확인할 때 이 토픽을 본다.

arda-raset 자체를 손대지 않기 위해, bus 큐를 subclass하거나 대체하지 않고
"콜백을 먼저 실행한 뒤 원본 큐에 그대로 전달"하는 극소 Tee 래퍼로 감싼다 —
raset의 세 워커 스레드는 원래 큐와 100% 동일하게 동작한다(get()으로 값을
훔쳐가지 않는다). 콜백을 원본 put()보다 먼저 실행하는 순서가 중요한 이유는
`_TeeQueue` docstring 참고.

⚠️ thermal_pending_timeout: radar_worker가 열화상 verdict를 기다리는
최대 시간인데, raset 원본(main.py)도 이 값과 `dwell_seconds`의 기본값이
똑같이 10.0이라 열화상이 실제 관찰을 마치기 *전에* 레이더가 먼저 포기해
verdict가 유실되는 버그가 있었다(Jetson 실기 레이더+열화상으로 재현
확인 — 아래 `_start_raset()`의 계산부 주석 참고). 이 노드는 명시적으로
값을 주지 않으면 자동으로 `dwell_seconds + dwell_margin_seconds`(기본
30.0)를 써서 넉넉한 마진을
보장한다(실측상 초과분이 give-up 경로에서 5~9.5초, 열원이 화면 가장자리
에서 계속 "움직이는" 채로 오래 끄는 경로에서는 40초 이상까지 널뛰어
15초 마진도 부족했음 — 완전한 상한 보장은 아니니 `/arda/tracker/
thermal_status`로 실제 소요시간을 보면서 필요하면 더 늘릴 것).
"""
import json
import os
import sys
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
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
        # 콜백을 원본 큐에 넣기 *전에* 먼저 실행한다. verdict_q처럼 콜백이
        # bus.pending_location 같은 다른 스레드와 공유하는 상태를 참조하는
        # 경우, 순서를 반대로 하면(원본 먼저) 그 사이에 원본 소비자(예:
        # radar_worker)가 즉시 get()해서 pending_location을 먼저 지워버릴
        # 여지가 생긴다 — 콜백을 먼저 실행하면 원본 소비자가 이 항목을 아직
        # 보지 못한 시점에 상태를 읽으므로 그 여지가 없다.
        try:
            self._on_put(item)
        except Exception:
            pass  # 콜백 실패가 raset 파이프라인 자체를 죽이면 안 됨
        self._inner.put(item)

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
        self.declare_parameter('model_path', '')          # 비우면 <thermal_dir>/models/s_yolo26.pt
        self.declare_parameter('confidence_threshold', 0.4)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('dwell_seconds', 10.0)
        # 사람 확정에 필요한 누적 매칭 횟수 — 연속일 필요는 없다(중간에
        # matched=False 프레임이 끼어도 카운트는 안 줄어듦). thermal_worker.py
        # 참고.
        self.declare_parameter('required_matches', 3)
        self.declare_parameter('settle_offset', 0.15)
        # thermal_pending_timeout과 servo_dwell_seconds가 자동(-1) 모드일 때
        # 공통으로 쓰는 안전 마진(초) — "dwell_seconds + 이 값"을 상한으로
        # 삼는다. 하나로 통일해둔 이유: 레이더(thermal_pending_timeout)와
        # 서보(servo_dwell_seconds) 둘 다 "열화상이 dwell_seconds를 넘겨
        # 실제로 얼마나 더 걸릴 수 있는가"라는 같은 불확실성(트리거 전파
        # 지연 + give-up 카운트다운이 계속 리셋되는 unbounded 케이스, 아래
        # thermal_pending_timeout 계산부 주석 참고)에 대한 여유이기 때문이다
        # — 실기 테스트로 40초 이상 걸리는 경우도 있어 기본을 30.0으로 잡되,
        # 환경에 따라 하나의 변수로 같이 조절할 수 있게 뺐다.
        self.declare_parameter('dwell_margin_seconds', 30.0)
        # -1(기본) = 자동 계산: dwell_seconds + dwell_margin_seconds. 양수를
        # 명시하면 그 값을 그대로 쓴다 — 반드시 dwell_seconds보다 커야 한다
        # (아래 _start_raset()의 thermal_pending_timeout 계산부 주석 참고.
        # 실측으로 확인된 버그: 이 값이 dwell_seconds와 같으면(예: 둘 다
        # 10.0) 열화상이 실제 관찰을 마치기 *전에* 레이더가 먼저 포기해
        # verdict가 통째로 유실된다).
        self.declare_parameter('thermal_pending_timeout', -1.0)
        # 서보가 dwell 중 마지막 조준 각도에서 얼마나 버틸지(초) — 열화상
        # 트리거 후 관찰 시간(dwell_seconds, 위)과는 별개 값이다. 원래는
        # <servo_dir>/config/settings.yaml의 servo.dwell_seconds로만 바꿀 수
        # 있었는데, 여기서도 자유롭게 조정할 수 있도록 파라미터로 노출한다.
        #
        # -1(기본) = 자동 계산: max(yaml 값, dwell_seconds + dwell_margin_seconds).
        # yaml 기본값(10.0)을 무조건 신뢰하지 않는 이유 — 서보의 dwell 연장은
        # "이번 프레임에 뭔가라도(사람 모양이 아니어도) 검출됐을 때"만
        # 일어나는데, 열화상이 몇 초간 아무것도 못 찾으면(아직 자기
        # dwell_seconds=30초는 안 지났는데도) 서보가 자기 폴백 타이머
        # (10초)로 먼저 포기해버리는 실측 버그가 있었다 — "매칭시도" 로그는
        # grid_xy 유무와 무관하게 매 프레임 찍히므로, 로그가 계속 나온다고
        # 서보가 보정을 받고 있다는 뜻은 아니다(thermal_pending_timeout과
        # 같은 종류의 문제). 0 이상을 명시하면 그 값을 그대로 쓰되(사용자
        # 의도 존중), dwell_seconds보다 작으면 경고만 남긴다.
        #
        # ⚠️ 또한 반드시 열화상 프레임 주기(FRAME_INTERVAL_S, 기본 0.5초)의
        # 2배 이상이어야 한다 — arda_servo.controller.ServoController.step()이
        # 열화상 보정(ThermalPan)을 받을 때마다 "지금부터 이 값만큼" dwell을
        # 연장하는 방식이라(controller.py의 "추적 연장" 로그 참고), 이 값이
        # 프레임 주기보다 짧으면 다음 보정이 오기 전에 dwell이 먼저 끝나
        # 버린다(위 자동 계산이면 사실상 항상 만족됨). 아래 _start_raset()
        # 에서 두 조건을 모두 검사해 위반 시 로그를 남긴다.
        self.declare_parameter('servo_dwell_seconds', -1.0)
        self.declare_parameter('radar_cli_port', '/dev/ttyUSB0')
        self.declare_parameter('radar_data_port', '/dev/ttyUSB1')
        self.declare_parameter('radar_settings', '')      # 비우면 <radar_dir>/config/settings.yaml
        self.declare_parameter('radar_profile', '')       # 비우면 <radar_dir>/config/profiles/xwr68xx_AOP_profile_short_range.cfg
        self.declare_parameter('servo_config', '')        # 비우면 <servo_dir>/config/settings.yaml

        # ════════════════════════════════════════════════════════════════
        # 센서 시각화 on/off — 웹 화면에 열화상/레이더 데이터를 띄울지 여부.
        # 끄면 해당 토픽 발행 자체를 건너뛰어 ROS/웹 대역폭을 아낀다. 사람
        # 확정 스냅샷(detection_confirmed)은 이 스위치와 무관하게 항상 발행됨.
        # ════════════════════════════════════════════════════════════════
        self.declare_parameter('enable_thermal_view', True)
        self.declare_parameter('enable_radar_view', True)
        self._enable_thermal_view = bool(self.get_parameter('enable_thermal_view').value)
        self._enable_radar_view = bool(self.get_parameter('enable_radar_view').value)

        # raset main.py의 --show-thermal과 동일 — 관찰 중 열화상 컬러맵을
        # 로컬 X11 창(cv2.imshow)에도 띄운다. DISPLAY 환경변수 없으면
        # main.py와 동일하게 자동으로 꺼진다(_start_raset()에서 확인).
        # enable_thermal_view(ROS 발행)와는 독립적 — 둘 다 켜면 웹 패널과
        # 로컬 창에 동시에 뜬다.
        self.declare_parameter('show_thermal', False)

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
        # 레이더 포인트클라우드/클러스터 스냅샷(시각화 전용) — waypoints_json과
        # 같은 이유로 JSON 문자열로 보낸다(임의 개수의 포인트 배열이라 표준
        # 메시지에 안 맞음).
        self._pub_radar_frame = self.create_publisher(
            String, '/arda/tracker/radar_frame', 10)
        # 서보 상태 스냅샷(시각화/rosbag 기록 전용) — 각도, dwell 중 여부,
        # 열화상이 붙잡고 있는지(thermal_engaged).
        self._pub_servo_status = self.create_publisher(
            String, '/arda/tracker/servo_status', 10)
        # 열화상 판정 진행 상태(시각화/rosbag 기록 전용) — 매 프레임
        # matched/match_count/confirmed. "사람 확정까지 얼마나
        # 가까워졌는지"를 dwell_seconds 튜닝하면서 바로 볼 수 있다.
        self._pub_thermal_status = self.create_publisher(
            String, '/arda/tracker/thermal_status', 10)

        self._cv_bridge = CvBridge() if CvBridge is not None else None
        self._last_thermal_frame = None  # _on_thermal_frame 이 매 프레임 갱신 (cv2 BGR ndarray)
        self._stop_event = threading.Event()
        self._threads: list = []
        self._bus = None  # _start_raset() 성공 시 설정 — 레이더 뷰 타이머가 읽음
        self._show_thermal = False  # _start_raset()이 DISPLAY 확인 후 설정
        self._orig_imshow = self._orig_waitkey = None

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
        # 경로 한 곳에서만 쓰인다 — ROS 발행을 얹기 위해 이 함수만 안전하게
        # 갈아끼운다(raset 코드 자체는 수정하지 않음). 원본 함수는 따로
        # 저장해뒀다가 show_thermal이 켜져 있으면 _on_thermal_frame 안에서
        # 그대로 호출해 로컬 창도 같이 띄운다(raset main.py의
        # --show-thermal과 동일한 동작 — DISPLAY 없으면 자동으로 꺼짐).
        import cv2
        self._cv2 = cv2
        self._orig_imshow = cv2.imshow
        self._orig_waitkey = cv2.waitKey
        cv2.imshow = self._on_thermal_frame
        cv2.waitKey = lambda *a, **k: -1

        show_thermal_param = bool(self.get_parameter('show_thermal').value)
        if show_thermal_param and not os.environ.get('DISPLAY'):
            self.get_logger().info('DISPLAY 환경변수가 없어 show_thermal을 무시합니다')
            show_thermal_param = False
        self._show_thermal = show_thermal_param

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

        site_cfg = load_settings(radar_settings_path).get('site', {})
        report_url = site_cfg.get('report_url', '')
        site_lat = site_cfg.get('lat')
        site_lon = site_cfg.get('lon')

        with open(servo_config_path, encoding='utf-8') as f:
            servo_cfg = yaml.safe_load(f)

        # servo_dwell_seconds 파라미터 처리 — 명시적으로 준 값(>=0)은 그대로
        # 쓰되 dwell_seconds보다 작으면 경고만 남긴다(사용자 의도 존중,
        # thermal_pending_timeout을 양수로 직접 줄 때와 같은 원칙). 자동(-1,
        # 기본)이면 yaml 값을 무조건 신뢰하지 않고 dwell_seconds+30.0과
        # 비교해 더 큰 쪽을 쓴다 — 그러지 않으면 yaml 기본값(10.0)이
        # dwell_seconds(예: 30.0)보다 작아서, 열화상이 프레임마다 아무것도
        # (사람 모양이 아닌 것조차) 못 찾는 구간이 10초만 지속돼도 서보
        # 자신의 폴백 타이머가 먼저 만료돼버린다 — 열화상은 아직 30초까지
        # 여유가 있는데 서보가 먼저 각도를 고정하고 트리거 대기로 돌아가서
        # "매칭시도 로그는 계속 찍히는데 서보는 dwell 초과로 끝남" 현상이
        # 실측됨(서보 dwell 연장은 grid_xy가 있는 프레임에서만 일어나고,
        # 매칭시도 로그는 grid_xy 유무와 무관하게 매 프레임 찍히므로 로그가
        # 계속 나온다고 서보가 보정을 받고 있다는 뜻은 아니다). 이 폴백
        # 타이머는 thermal_worker가 give_up/confirmed를 명시적으로 보내면
        # (controller.py의 _end_tracking) dwell 만료를 기다리지 않고 즉시
        # 끝나므로, 넉넉하게 잡아도 정상 종료가 늦어지지는 않는다.
        dwell_seconds_for_servo = float(self.get_parameter('dwell_seconds').value)
        dwell_margin_seconds = float(self.get_parameter('dwell_margin_seconds').value)
        servo_dwell_seconds_param = float(self.get_parameter('servo_dwell_seconds').value)
        yaml_servo_dwell_seconds = servo_cfg.get('servo', {}).get('dwell_seconds', 10.0)
        if servo_dwell_seconds_param >= 0.0:
            effective_servo_dwell_seconds = servo_dwell_seconds_param
            if effective_servo_dwell_seconds < dwell_seconds_for_servo:
                self.get_logger().warning(
                    f'servo_dwell_seconds({effective_servo_dwell_seconds}s)가 dwell_seconds'
                    f'({dwell_seconds_for_servo}s)보다 작습니다 — 열화상이 아직 관찰 중인데 '
                    f'서보가 먼저 포기하고 각도를 고정할 수 있습니다.')
        else:
            effective_servo_dwell_seconds = max(yaml_servo_dwell_seconds, dwell_seconds_for_servo + dwell_margin_seconds)
        servo_cfg.setdefault('servo', {})['dwell_seconds'] = effective_servo_dwell_seconds

        # 추가로, 열화상 프레임 주기(FRAME_INTERVAL_S)보다도 충분히(2배 이상)
        # 커야 한다 — grid_xy가 매 프레임 검출되는 정상 상황에서도 "추적
        # 연장"이 다음 보정 도착 전에 만료되지 않게 하기 위함. 위 자동 계산
        # (dwell_seconds+30.0)이면 사실상 항상 만족되지만, 명시적으로 아주
        # 작은 값을 준 경우를 대비해 별도로 검사한다.
        frame_interval_s = getattr(thermal_backend, 'FRAME_INTERVAL_S', 0.5)
        if effective_servo_dwell_seconds <= frame_interval_s:
            self.get_logger().error(
                f'servo_dwell_seconds({effective_servo_dwell_seconds}s)가 열화상 프레임 주기'
                f'({frame_interval_s}s) 이하입니다 — 서보가 열화상 관찰 도중에 dwell이 끝나'
                f'추적을 멈출 수 있습니다. servo_dwell_seconds를 늘리세요.')
        elif effective_servo_dwell_seconds < frame_interval_s * 2:
            self.get_logger().warning(
                f'servo_dwell_seconds({effective_servo_dwell_seconds}s)가 열화상 프레임 주기'
                f'({frame_interval_s}s)의 2배보다 작습니다 — 프레임 지연이 조금만 있어도 '
                f'추적 연장이 늦어 dwell이 만료될 수 있습니다.')

        bus = Bus()
        self._bus = bus  # _on_radar_view_timer()가 radar_frame_q를 읽는 데 씀
        # detection_trigger 발행: 원본 소비자(thermal_worker)는 그대로 두고
        # put() 시점에만 콜백을 얹는다.
        bus.trigger_q = _TeeQueue(bus.trigger_q, self._on_trigger)
        # absolute_pose 발행: verdict(person 확정 여부)가 나오는 순간
        # pending_location(그 판정을 기다리던 좌표)을 같이 스냅샷한다.
        # _TeeQueue.put()이 콜백을 원본 큐 put()보다 먼저 실행하므로,
        # radar_worker(원본 소비자)가 이 verdict를 get()해서
        # pending_location.clear()를 하기 전에 이 콜백이 항상 먼저 읽는다
        # (put()/get() 두 스레드 사이의 순서 보장 — 자세한 이유는 _TeeQueue
        # 클래스 docstring 참고).
        bus.verdict_q = _TeeQueue(bus.verdict_q, lambda v: self._on_verdict(v, bus))

        simulate_servo = bool(self.get_parameter('simulate_servo').value)
        simulate_thermal = bool(self.get_parameter('simulate_thermal').value)
        no_radar = bool(self.get_parameter('no_radar').value)
        use_yolo = bool(self.get_parameter('yolo').value)
        model_path_param = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        dwell_seconds = float(self.get_parameter('dwell_seconds').value)
        required_matches = int(self.get_parameter('required_matches').value)
        settle_offset = float(self.get_parameter('settle_offset').value)

        # thermal_pending_timeout: radar_worker(raset/radar_worker.py 146~169행)가
        # 열화상 verdict를 기다리는 최대 시간. 열화상의 실제 관찰 소요시간은
        # dwell_seconds 그 자체가 아니라 항상 그보다 더 걸린다 — 트리거 전파
        # 지연(~0.02~0.2s), give-up 판정이 다음 프레임 루프에서만 감지되는
        # 지연(최대 FRAME_INTERVAL_S=0.5s, thermal-camera/src/thermal_main.py)
        # 뿐 아니라, thermal_worker._run_observation()의
        # `if moving: give_up_deadline = None` 로직 때문에 열원이 계속
        # settle_offset 밖에서 움직이는 동안은 dwell 카운트다운 자체가
        # 계속 리셋된다 — 즉 실제 소요시간에는 이론적 상한이 없다. 이 값이
        # dwell_seconds와 같거나 비슷하면(예: 원본 raset main.py 기본값처럼
        # 둘 다 10.0) radar_worker가 먼저 timeout 처리를 해버려
        # bus.pending_location을 지우고, 뒤늦게 도착한 열화상 verdict는
        # (person 확정이든 기각이든) 조용히 버려진다 — absolute_pose가
        # 발행되지 않는 근본 원인이었다. 실제 Jetson 하드웨어(라이브 레이더+
        # 열화상)로 재현 확인한 결과 dwell_seconds 대비 초과분이 트리거마다
        # 5~9.5초(give-up 경로) 정도였는데, 열원이 화면 가장자리 근처에서
        # 계속 "움직이는" 상태로 오래 머무는 경우(=give-up 카운트다운이
        # 계속 리셋되는 경우)는 confirmed=true까지 40초 이상 걸리는 것도
        # 실측됨(15초 마진으로는 부족해서 재발 — thermal_status 토픽의
        # frame_number/match_count로 실측 확인). 그래서 여유를 15초가
        # 아니라 30초로 더 크게 잡는다. 이래도 열원이 병적으로 계속
        # 흔들리며 settle_offset 안으로 절대 안 들어오면 이론적으로는
        # 여전히 timeout이 먼저 올 수 있다(위 unbounded 특성 때문 —
        # raset 자체를 고치지 않는 한 완전히는 못 막음). 실기 테스트
        # 중이라면 카메라 중앙에 빠르게 정지 상태로 들어오는 편이 이
        # "움직이는 채로 오래 끄는" 경로를 피하는 가장 확실한 방법이다.
        # 파라미터를 양수로 명시하면 그 값을 그대로 쓴다.
        thermal_pending_timeout_param = float(self.get_parameter('thermal_pending_timeout').value)
        thermal_pending_timeout = (
            thermal_pending_timeout_param if thermal_pending_timeout_param > 0.0
            else dwell_seconds + dwell_margin_seconds
        )
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
                    model_path = model_path_param or str(thermal_dir / 'models' / 's_yolo26.pt')
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
                    dwell_seconds, required_matches, settle_offset, report_url,
                    True,  # show=True — 위에서 cv2.imshow를 ROS 발행으로 갈아끼웠으므로 항상 켠다
                    site_lat, site_lon,
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

        # 레이더 시각화 — bus.radar_frame_q(LatestQueue, raset/radar_worker.py가
        # 매 프레임 최신 포인트클라우드/클러스터 스냅샷을 채워둠)를 5Hz로
        # 폴링해 JSON으로 발행한다. enable_radar_view=false면 폴링만 하고
        # 발행은 건너뛴다(레이더 워커 쪽 오버헤드는 원래 미미해 굳이 안 막음).
        self.create_timer(0.2, self._on_radar_view_timer)
        # 서보 상태 — on/off 스위치 없이 항상 5Hz로 발행(가벼운 JSON 몇
        # 필드라 오버헤드 무시할 만함). rosbag 기록/타이밍 분석용.
        self.create_timer(0.2, self._on_servo_status_timer)
        # 열화상 판정 진행 상태 — 마찬가지로 항상 5Hz 폴링(열화상 프레임
        # 자체는 2Hz라 놓치지 않음). rosbag 기록/타이밍 분석용.
        self.create_timer(0.2, self._on_thermal_status_timer)

        self.get_logger().info(
            f'arda-raset 서브시스템 시작됨 (thermal={thermal_started}, radar_dir={radar_dir}, servo_dir={servo_dir})')

    # ------------------------------------------------------------------
    # 레이더 시각화 — bus.radar_frame_q → ROS 발행
    # ------------------------------------------------------------------
    def _on_radar_view_timer(self):
        if not self._enable_radar_view or self._bus is None:
            return
        frame = self._bus.radar_frame_q.get(timeout=0)
        if frame is None:
            return
        msg = String()
        msg.data = json.dumps(frame)
        self._pub_radar_frame.publish(msg)

    # ------------------------------------------------------------------
    # 서보 시각화/rosbag 기록 — bus.servo_status_q → ROS 발행
    # ------------------------------------------------------------------
    def _on_servo_status_timer(self):
        if self._bus is None:
            return
        status = self._bus.servo_status_q.get(timeout=0)
        if status is None:
            return
        msg = String()
        msg.data = json.dumps(status)
        self._pub_servo_status.publish(msg)

    # ------------------------------------------------------------------
    # 열화상 판정 진행 상태 — bus.thermal_status_q → ROS 발행
    # ------------------------------------------------------------------
    def _on_thermal_status_timer(self):
        if self._bus is None:
            return
        status = self._bus.thermal_status_q.get(timeout=0)
        if status is None:
            return
        msg = String()
        msg.data = json.dumps(status)
        self._pub_thermal_status.publish(msg)

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
        # _last_thermal_frame은 detection_confirmed 스냅샷(_on_verdict)이 항상
        # 필요로 하므로 enable_thermal_view와 무관하게 매 프레임 갱신한다 —
        # 이 스위치는 연속 스트림(thermal_image) 발행 여부만 결정한다.
        self._last_thermal_frame = image

        # show_thermal(--show-thermal과 동일) — 원본 cv2.imshow/waitKey를
        # 그대로 호출해 로컬 X11 창도 띄운다. enable_thermal_view(ROS 발행)
        # 와는 완전히 독립적으로 동작한다.
        if self._show_thermal:
            try:
                self._orig_imshow(_window_name, image)
                self._orig_waitkey(1)
            except Exception as exc:
                self.get_logger().warn(f'열화상 로컬 창 표시 실패: {exc}')

        if not self._enable_thermal_view or self._cv_bridge is None:
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
