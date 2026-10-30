import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class OpticalFlowPublisher(Node):
    def __init__(self):
        super().__init__('optical_flow_publisher')
        self.get_logger().info("Starting OpticalFlowPublisher...")

        # ===== FPS 파라미터 =====
        self.alpha = 0.2                 # EMA 계수(클수록 안정)
        self.report_interval = 2.0       # 로그 출력 주기 [s]
        self.prev_arrival = None         # 마지막 수신 시각(rclpy Clock)
        self.prev_header = None          # 마지막 header.stamp
        self.ema_arrival = 0.0           # 수신간격 기반 FPS(EMA)
        self.ema_header = 0.0            # header 기반 FPS(EMA)
        self.last_report = self.get_clock().now()

        # Subscriber
        self.subscription = self.create_subscription(
            Image, '/image_raw', self.image_callback, 10
        )

        # Publishers
        self.image_optical_vector_publisher = self.create_publisher(
            Image, '/image_optical_vector', 10
        )

        # ===== [ADDED] BEV 이미지 확인용 Publisher 시작 =====
        self.image_bev_publisher = self.create_publisher(
            Image, '/image_bev', 10
        )
        # ===== [ADDED] BEV 이미지 확인용 Publisher 끝 =====

        self.bridge = CvBridge()
        self.prev_gray = None

                # ===== [ADDED] BEV 파라미터 시작 =====

        # BEV 출력 영상 크기
        self.bev_width = 640
        self.bev_height = 480

        # 원본 이미지의 지면 사다리꼴 좌표
        # 좌표 순서:
        # 좌상단 → 우상단 → 우하단 → 좌하단
        #
        # 주의: 아래 좌표는 640 × 480 영상 기준 예시값
        self.src_points = np.float32([
            [180.0, 180.0],   # 좌상단
            [460.0, 180.0],   # 우상단
            [630.0, 475.0],   # 우하단
            [10.0,  475.0]    # 좌하단
        ])

        # BEV 출력 영상의 직사각형 좌표
        self.dst_points = np.float32([
            [100.0, 0.0],                            # 좌상단
            [self.bev_width - 100.0, 0.0],           # 우상단
            [self.bev_width - 100.0,
             self.bev_height - 1.0],                 # 우하단
            [100.0, self.bev_height - 1.0]           # 좌하단
        ])

        # 호모그래피 행렬 계산
        # 카메라 위치가 고정되어 있으면 최초 한 번만 계산
        self.bev_matrix = cv2.getPerspectiveTransform(
            self.src_points,
            self.dst_points
        )

        # ===== [ADDED] BEV 파라미터 끝 =====

        # ▼ 추가: EMA 상태 저장용 딕셔너리 & 컷오프 주파수
        self.mag_state = {}   # key: (y, x) → 직전 EMA 값
        self.fc = 4.0         # 차단 주파수(Hz)

    # ----- FPS 내부 유틸 -----
    def _ema_update(self, ema_val, sample):
        return sample if ema_val == 0.0 else (self.alpha * ema_val + (1.0 - self.alpha) * sample)

    def _maybe_report_fps(self):
        now = self.get_clock().now()
        if (now - self.last_report).nanoseconds >= int(self.report_interval * 1e9):
            if self.ema_header > 0:
                self.get_logger().info(
                    f"FPS arrive/header: {self.ema_arrival:.2f} / {self.ema_header:.2f}"
                )
            else:
                self.get_logger().info(f"FPS arrive: {self.ema_arrival:.2f}")
            self.last_report = now

    def image_callback(self, msg: Image):
        now = self.get_clock().now()
        # ----- FPS(1) 도착 시각 기준 -----
        if self.prev_arrival is not None:
            dt = (now - self.prev_arrival).nanoseconds / 1e9
            if dt > 0:
                self.ema_arrival = self._ema_update(self.ema_arrival, 1.0 / dt)
        self.prev_arrival = now

        # ----- FPS(2) header.stamp 기준 -----
        if msg.header.stamp.sec != 0 or msg.header.stamp.nanosec != 0:
            t = Time.from_msg(msg.header.stamp)
            if self.prev_header is not None:
                dt_h = (t - self.prev_header).nanoseconds / 1e9
                if dt_h > 0:
                    self.ema_header = self._ema_update(self.ema_header, 1.0 / dt_h)
            self.prev_header = t
        
        # Convert ROS Image to OpenCV image (BGR)
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        beforeBEV = False
        if beforeBEV:
            current_gray = cv2.cvtColor(current_frame, cv2.COLOR_BGR2GRAY)
        else:
        # ===== [ADDED] 원본 카메라 영상에 BEV 적용 시작 =====
            bev_frame = cv2.warpPerspective(
                current_frame,
                self.bev_matrix,
                (self.bev_width, self.bev_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0)
            )
        # ===== [ADDED] 원본 카메라 영상에 BEV 적용 끝 =====

        # ===== [ADDED] BEV 영상 토픽 발행 시작 =====
            bev_msg = self.bridge.cv2_to_imgmsg(
                bev_frame,
                encoding='bgr8'
            )
            bev_msg.header = msg.header
            self.image_bev_publisher.publish(bev_msg)
        # ===== [ADDED] BEV 영상 토픽 발행 끝 =====

        # ===== [CHANGED] 원본 영상이 아니라 BEV 영상을 Gray로 변환 =====
            current_gray = cv2.cvtColor(
                bev_frame,
                cv2.COLOR_BGR2GRAY
            )

        # ===== [ADDED] 원본 카메라 영상에 BEV 적용 시작 =====
            bev_frame = cv2.warpPerspective(
                current_frame,
                self.bev_matrix,
                (self.bev_width, self.bev_height),
                flags=cv2.INTER_LINEAR,
                borderMode=cv2.BORDER_CONSTANT,
                borderValue=(0, 0, 0)
            )
        # ===== [ADDED] 원본 카메라 영상에 BEV 적용 끝 =====

        # If this is the first frame, just store and return
        if self.prev_gray is None:
            self.prev_gray = current_gray
            # 첫 프레임에도 현재까지의 FPS를 오버레이만 해두자
            out_img = current_frame.copy()
            self._draw_fps_overlay(out_img)
            flow_vec_msg = self.bridge.cv2_to_imgmsg(out_img, encoding='bgr8')
            flow_vec_msg.header = msg.header
            self.image_optical_vector_publisher.publish(flow_vec_msg)
            return

        # 1. Optical Flow (Farneback)
        flow = cv2.calcOpticalFlowFarneback(
            self.prev_gray,      # previous frame (gray)
            current_gray,        # current frame  (gray)
            None,                # flow (output)
            0.5,                 # pyr_scale
            3,                   # levels
            15,                  # winsize
            3,                   # iterations
            5,                   # poly_n
            1.2,                 # poly_sigma
            0                    # flags
        )

        # Publish original image with optical flow vectors (arrows)
        flow_vec_img = current_frame.copy()
        self.draw_flow_vectors(flow_vec_img, flow, step=16)

        # ==== [ADDED] 임계값을 넘긴 벡터들을 이웃으로 묶어 '네모 박스'로 표시 ====
        flow_vec_img = self.draw_threshold_boxes(
            img=flow_vec_img,
            vx=flow[..., 0], vy=flow[..., 1],
            thresh=0.8,        # 벡터 크기 임계값(픽셀/프레임) - 튜닝
            step=8,            # 화살표 간격과 동일 권장
            dilate_px=16,       # 연결 강도(픽셀) - step/2~step 권장
            min_points=6,      # 박스 내 최소 유효 포인트 수
            min_box_area=200,  # 너무 작은 박스 제거 (픽셀^2)
            merge=True,        # NMS로 겹치는 박스 합치기
            iou_thr=0.2,       # NMS IoU 임계
            color=(0, 200, 255),
            thickness=1,
            draw_label=True    # 평균 크기/포인트수 라벨 표시
        )
        # ==== [ADDED] 끝 ====

        # 3) ===== FPS 오버레이 =====
        self._draw_fps_overlay(flow_vec_img)

        # Publish
        flow_vec_msg = self.bridge.cv2_to_imgmsg(flow_vec_img, encoding='bgr8')
        flow_vec_msg.header = msg.header
        self.image_optical_vector_publisher.publish(flow_vec_msg)

        # Update prev_gray for next call
        self.prev_gray = current_gray

        # 주기적 로그
        self._maybe_report_fps()

    def _draw_fps_overlay(self, frame):
        """화면 좌상단에 arrive/header FPS 오버레이"""
        txt = f"FPS(arrive): {self.ema_arrival:.2f}"
        if self.ema_header > 0:
            txt += f" | header: {self.ema_header:.2f}"
        cv2.putText(frame, txt, (10, 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (50, 220, 50), 2, cv2.LINE_AA)
    
    def draw_flow_vectors(self, frame, flow, step=8):
        h, w = flow.shape[:2]

        for y in range(0, h, step):
            for x in range(0, w, step):
                # Optical Flow (Δx, Δy)
                fx, fy = flow[y, x]
                end_x = int(x + fx)
                end_y = int(y + fy)
                
                # 정확식: alpha = 1 - exp(-2*pi*fc/fs)
                alpha = float(1.0 - np.exp(-2.0 * np.pi * self.fc / 30))
                
                # 화살표 그리기
                scalar_vector = float(np.hypot(fx, fy))
                # --- 추가: 좌표별 EMA 상태 갱신(시간적 피드백) ---
                key = (y, x)
                prev = self.mag_state.get(key, scalar_vector)      # 초기엔 현재값으로 시드
                scalar_vector_f = prev + alpha * (scalar_vector - prev)        # EMA
                self.mag_state[key] = scalar_vector_f

                if scalar_vector_f >= 0.1:
                    cv2.arrowedLine(
                    frame, (x, y), (end_x, end_y), color=(0, 0, 255), thickness=1, tipLength=0.4
                    )
                else:
                    cv2.arrowedLine(
                    frame, (x, y), (end_x, end_y), color=(0, 255, 0), thickness=1, tipLength=0.4
                    )          

    # ==== [ADDED] IoU 계산 (NMS용) ====
    @staticmethod
    def _iou_xywh(a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh

        inter_x1 = max(ax, bx); inter_y1 = max(ay, by)
        inter_x2 = min(ax2, bx2); inter_y2 = min(ay2, by2)
        iw = max(0, inter_x2 - inter_x1); ih = max(0, inter_y2 - inter_y1)
        inter = iw * ih
        if inter == 0:
            return 0.0
        area_a = aw * ah
        area_b = bw * bh
        return inter / float(area_a + area_b - inter)

    # ==== [ADDED] 간단 NMS ====
    def _nms_boxes(self, boxes, scores, iou_thr=0.3):
        if not boxes:
            return []
        idxs = np.argsort(scores)[::-1]  # 점수 내림차순
        keep = []
        while len(idxs) > 0:
            i = idxs[0]
            keep.append(i)
            rest = []
            for j in idxs[1:]:
                if self._iou_xywh(boxes[i], boxes[j]) <= iou_thr:
                    rest.append(j)
            idxs = np.array(rest, dtype=int) if rest else np.array([], dtype=int)
        return [boxes[i] for i in keep]

    # ==== [ADDED] 임계 초과 이웃을 네모 박스로 묶어 그리기 ====
    def draw_threshold_boxes(
        self,
        img: np.ndarray,
        vx: np.ndarray, vy: np.ndarray,
        thresh: float = 0.8,
        step: int = 8,
        dilate_px: int = 16,
        min_points: int = 16,
        min_box_area: int = 200,
        merge: bool = True,
        iou_thr: float = 0.2,
        color: tuple[int, int, int] = (0, 200, 255),
        thickness: int = 1,
        draw_label: bool = True
    ) -> np.ndarray:
        """
        1) |v| >= thresh 인 grid-step 포인트만 마스크에 표시
        2) 팽창으로 인접 포인트 연결
        3) 컨투어/라벨링으로 그룹 → 각 그룹을 axis-aligned 박스로 표시
        4) (옵션) NMS로 겹치는 박스 정리
        """
        h, w = img.shape[:2]
        mag = np.hypot(vx, vy).astype(np.float32)

        # (A) grid-step 포인트만 표본화하여 마스크 생성 (빠르고 깔끔)
        mask = np.zeros((h, w), dtype=np.uint8)
        for y in range(0, h, step):
            for x in range(0, w, step):
                if mag[y, x] >= thresh:
                    mask[y, x] = 255

        # (B) 팽창으로 이웃 연결 강화
        k = max(1, dilate_px)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask_conn = cv2.dilate(mask, kernel, iterations=1)

        # (C) 컨투어로 그룹 찾기
        contours, _ = cv2.findContours(mask_conn, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        boxes, scores = [], []
        for cnt in contours:
            if len(cnt) < 3:
                continue
            x, y, w_box, h_box = cv2.boundingRect(cnt)
            if w_box * h_box < min_box_area:
                continue

            # 박스 내부에서 '실제 유효 포인트' 수와 평균 크기 계산
            sub_mask = mask[y:y+h_box, x:x+w_box]
            pts_count = int(cv2.countNonZero(sub_mask))
            if pts_count < min_points:
                continue

            # 점수: 평균 magnitude (원하면 최대값, 합계로 바꿔도 됨)
            sub_mag = mag[y:y+h_box, x:x+w_box]
            mean_mag = float(sub_mag[sub_mask > 0].mean()) if pts_count > 0 else 0.0

            boxes.append((x, y, w_box, h_box))
            scores.append(mean_mag)

        # (D) NMS로 중복 박스 제거
        if merge and boxes:
            boxes = self._nms_boxes(boxes, scores, iou_thr=iou_thr)

        # (E) 박스 그리기
        out = img.copy()
        for (x, y, w_box, h_box) in boxes:
            cv2.rectangle(out, (x, y), (x + w_box, y + h_box), color, thickness)
            if draw_label:
                roi_mask = mask[y:y+h_box, x:x+w_box]
                roi_mag = mag[y:y+h_box, x:x+w_box]
                cnt_pts = int(cv2.countNonZero(roi_mask))
                mean_mag = float(roi_mag[roi_mask > 0].mean()) if cnt_pts > 0 else 0.0
                label = f"{mean_mag:.2f} | n={cnt_pts}"
                cv2.putText(out, label, (x, max(0, y - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)
        return out

def main():
    rclpy.init()
    node = OpticalFlowPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
