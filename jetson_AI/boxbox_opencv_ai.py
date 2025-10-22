# debug_live_new7.py
import cv2
import numpy as np
import os
import argparse
import pytesseract
import onnxruntime as ort
from Levenshtein import distance
import time
import socket  # TCP/IP capture command
import threading  # TCP/IP capture command
import struct
import re

# --- ⭐ 설정 부분 ⭐ ---
# YOLOv8 ONNX 모델 및 관련 설정
ONNX_MODEL_PATH = "best.onnx"
YOLO_CLASS_NAMES = ['care', 'fragile', 'water', 'upside']

# OCR 관련 설정
OCR_DICTIONARY = ["서울", "부산", "대전"]

# 카메라 및 탐지 설정
CAMERA_INDEX = 2
CONF_THRESHOLD = 0.4  # AI가 스티커를 판단하는 확신도 임계값
IOU_THRESHOLD = 0.5
DETECTION_INTERVAL = 0.5

# --- ⭐ TCP/IP 설정 ⭐ ---
TCP_HOST = "192.168.0.2"  # 외부 TCP 서버 IP
TCP_PORT = 9190           # 외부 TCP 서버 포트
TCP_CLIENT_NAME = "jetson"  # 클라이언트 ID

# --- TCP/IP capture 클라이언트 클래스 (기존 유지) ---
class CaptureCommandClient:
    """외부 TCP/IP 서버에 연결해 'capture' 명령을 받고 결과를 전송합니다."""

    def __init__(
        self,
        host,
        port,
        reconnect_interval=5,
        client_name="jetson",
        default_target="ALLMSG",
    ):
        self.host, self.port = host, port
        self.reconnect_interval = reconnect_interval
        self.client_name = client_name
        self.default_target = default_target
        self.capture_event = threading.Event()
        self.running = True
        self._socket = None
        self._socket_lock = threading.Lock()
        self._receiver_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self._receiver_thread.start()

    def _connection_loop(self):
        while self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(10)
                sock.connect((self.host, self.port))
                sock.settimeout(None)
                with self._socket_lock:
                    self._socket = sock
                print(f"[TCP] Connected to server {self.host}:{self.port}")
                if self._send_handshake(sock):
                    self._receive_loop(sock)
            except Exception as exc:
                if self.running:
                    print(f"[TCP] Connection attempt failed: {exc}")
            finally:
                with self._socket_lock:
                    if self._socket:
                        try:
                            self._socket.close()
                        except OSError:
                            pass
                        self._socket = None
            if self.running:
                time.sleep(self.reconnect_interval)

    def _send_handshake(self, sock):
        # 핸드셰이크에서 비밀번호 부분 삭제
        handshake = f"{self.client_name}\n"
        try:
            sock.sendall(handshake.encode("utf-8"))
            print(f"[TCP] Handshake sent: {handshake.strip()}")
            return True
        except OSError as exc:
            print(f"[TCP] Failed to send handshake: {exc}")
            return False

    def _receive_loop(self, sock):
        buffer = b""
        try:
            while self.running:
                data = sock.recv(1024)
                if not data:
                    print("[TCP] Server closed the connection.")
                    break
                buffer += data
                while b"\n" in buffer:
                    line, buffer = buffer.split(b"\n", 1)
                    decoded_line = line.decode("utf-8", errors="ignore").strip()
                    if not decoded_line:
                        continue

                    print(f"[TCP] Received: {decoded_line}")  # 수신된 모든 메시지를 로그로 출력

                    # 메시지 파싱: "<client>|capture|t"
                    parts = decoded_line.lower().split('|')
                    if (len(parts) >= 3 and
                        parts[0] == self.client_name.lower() and
                        parts[1] == 'capture' and
                        parts[2] == 't'):
                        print("[TCP] Valid 'capture' command received.")
                        self.capture_event.set()

        except OSError as exc:
            if self.running:
                print(f"[TCP] Connection error: {exc}")
        finally:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass

    def is_connected(self):
        with self._socket_lock:
            return self._socket is not None

    def check_and_clear_capture(self):
        if self.capture_event.is_set():
            self.capture_event.clear()
            return True
        return False

    @staticmethod
    def send_image(client: 'CaptureCommandClient', image, target="QtClient"):
        """이미지를 JPEG로 인코딩 후 {IMG} 프로토콜로 전송"""
        ret, buf = cv2.imencode('.jpg', image)
        if not ret:
            print("[IMG] 인코딩 실패")
            return
        data = buf.tobytes()
        header = f"{{IMG}}{len(data)}\n"
        with client._socket_lock:
            sock = client._socket
        if not sock:
            print("[IMG] 서버 연결 안 됨")
            return
        try:
            sock.sendall(header.encode("utf-8"))
            sock.sendall(data)
            print(f"[IMG] 이미지 {len(data)} bytes 전송 완료")
        except OSError as e:
            print(f"[IMG] 전송 오류: {e}")

    def shutdown(self):
        self.running = False
        with self._socket_lock:
            sock = self._socket
            self._socket = None
        if sock:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
        self.capture_event.set()
        if self._receiver_thread.is_alive():
            self._receiver_thread.join(timeout=2)


def nms_xywh(boxes, scores, iou_thres=0.5):
    """
    boxes: list/np.ndarray of [x, y, w, h]
    scores: list/np.ndarray of confidence scores
    return: kept indices (list of int)
    """
    if len(boxes) == 0:
        return []

    boxes = np.asarray(boxes, dtype=np.float32)
    scores = np.asarray(scores, dtype=np.float32)

    x1 = boxes[:, 0]
    y1 = boxes[:, 1]
    x2 = boxes[:, 0] + boxes[:, 2]
    y2 = boxes[:, 1] + boxes[:, 3]

    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]

    keep = []
    while order.size > 0:
        i = int(order[0])
        keep.append(i)
        if order.size == 1:
            break
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])

        w = np.maximum(0.0, xx2 - xx1)
        h = np.maximum(0.0, yy2 - yy1)
        inter = w * h
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)

        remaining = np.where(iou <= iou_thres)[0]
        order = order[remaining + 1]
    return keep


# === 암호화된 결과 전송 유틸 (기존 유지) ===
def send_encoded_results(client, ocr_text, det_classes, target="qt"):
    """
    OCR과 탐지 결과를 새로운 규격('qt|det|ocr,det')에 맞춰 전송합니다.
    """
    ocr_map = {"서울": "1", "대전": "2", "부산": "3"}
    det_map = {"fragile": "a", "care": "b", "water": "c", "upside": "d"}

    ocr_code = ocr_map.get(ocr_text, "0")

    if det_classes:
        encoded_dets = [det_map[cls] for cls in det_classes if cls in det_map]
        det_codes_str = "".join(sorted(encoded_dets))
    else:
        det_codes_str = ""

    combined_values = f"{ocr_code},{det_codes_str}"
    line_to_send = f"{target}|det|{combined_values}\n"

    payload = line_to_send.encode("utf-8", "replace")
    with client._socket_lock:
        sock = client._socket
    if not sock:
        print("[TCP] Unable to send message: not connected to server.")
        return
    try:
        sock.sendall(payload)
        print(f"[TCP] Sent Encoded Result: {line_to_send.strip()}")
    except OSError as exc:
        print(f"[TCP] Failed to send message: {exc}")


# --- YOLOv8 탐지기 클래스 (기존 유지) ---
class YOLOv8_Detector:
    def __init__(self, model_path, confidence_thres=0.5, iou_thres=0.5):
        self.confidence_thres, self.iou_thres = confidence_thres, iou_thres
        try:
            self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])
            print("YOLOv8 ONNX 모델 로드 성공.")
        except Exception as e:
            print(f"오류: YOLOv8 ONNX 모델 로드 실패: {e}"); exit()
        self.get_input_details()
        self.get_output_details()

    def get_input_details(self):
        model_inputs = self.session.get_inputs()
        self.input_names = [model_inputs[i].name for i in range(len(model_inputs))]
        self.input_shape = model_inputs[0].shape
        self.input_height, self.input_width = self.input_shape[2], self.input_shape[3]

    def get_output_details(self):
        model_outputs = self.session.get_outputs()
        self.output_names = [model_outputs[i].name for i in range(len(model_outputs))]

    def detect_objects(self, image):
        input_tensor = self.prepare_input(image)
        start_time = time.time()
        outputs = self.session.run(self.output_names, {self.input_names[0]: input_tensor})
        end_time = time.time()
        inference_time = (end_time - start_time) * 1000
        print(f"   ===> Inference Time: {inference_time:.2f} ms")
        return self.process_output(outputs)

    def prepare_input(self, image):
        self.img_height, self.img_width = image.shape[:2]
        input_img = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        scale = min(self.input_width / self.img_width, self.input_height / self.img_height)
        resized_w, resized_h = int(self.img_width * scale), int(self.img_height * scale)
        resized_img = cv2.resize(input_img, (resized_w, resized_h))
        padded_img = np.full((self.input_height, self.input_width, 3), 114, dtype=np.uint8)
        padded_img[:resized_h, :resized_w] = resized_img
        padded_img = padded_img.transpose(2, 0, 1)
        input_tensor = padded_img.astype(np.float32) / 255.0
        return np.expand_dims(input_tensor, axis=0)

    def process_output(self, output):
        predictions = np.squeeze(output[0]).T
        scores = np.max(predictions[:, 4:], axis=1)
        predictions = predictions[scores > self.confidence_thres, :]
        scores = scores[scores > self.confidence_thres]
        if len(scores) == 0: return [], [], []
        class_ids = np.argmax(predictions[:, 4:], axis=1)
        x, y, w, h = predictions[:, 0], predictions[:, 1], predictions[:, 2], predictions[:, 3]
        x_scale, y_scale = self.img_width / self.input_width, self.img_height / self.input_height
        x1, y1 = (x - w / 2) * x_scale, (y - h / 2) * y_scale
        x2, y2 = (x + w / 2) * x_scale, (y + h / 2) * y_scale
        boxes = np.array(list(zip(x1, y1, x2, y2)))
        boxes_xywh = np.array(list(zip(x1, y1, x2 - x1, y2 - y1)))
        indices = nms_xywh(boxes_xywh, scores, self.iou_thres)
        if len(indices) > 0:
            return boxes[indices], scores[indices], class_ids[indices]
        else:
            return [], [], []


# =====================
#  새 후보 검출 함수들
# =====================

def _find_contours_external(bin_img):
    """OpenCV 3/4 호환적으로 외곽 컨투어만 반환"""
    res = cv2.findContours(bin_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if len(res) == 3:
        _, contours, _ = res
    else:
        contours, _ = res
    return contours

def find_invoice_by_black_border(image):
    """
    화면 전체에서 '두꺼운 검은 테두리'를 가진 직사각형을 찾는다.
    반환: [(x,y,w,h)] 또는 []
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (3,3), 0)

    # 경계 강조 후 끊긴 선 메우기
    edges = cv2.Canny(blur, 50, 150)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5,5))
    closed = cv2.morphologyEx(edges, cv2.MORPH_CLOSE, kernel, iterations=2)

    contours = _find_contours_external(closed)

    H, W = gray.shape[:2]
    img_area = H * W
    best = None
    best_score = -1

    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        area = w*h
        # 프레임 대비 너무 작거나 너무 큰 후보 제외
        if area < 0.01*img_area or area > 0.6*img_area:
            continue

        # 사각형성 & 4-점 근사
        peri = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.02*peri, True)
        if len(approx) != 4:
            continue

        rectangularity = cv2.contourArea(c) / float(max(area, 1))
        if rectangularity < 0.65:
            continue

        # '두꺼운 검은 테두리' 점수화: 바깥-안쪽 마스크의 고리(ring) 분석
        outer_mask = np.zeros_like(gray, np.uint8)
        rect = cv2.minAreaRect(c)
        pts = cv2.boxPoints(rect).astype(np.int32).reshape((-1,1,2))
        cv2.drawContours(outer_mask, [pts], -1, 255, thickness=-1)

        # 크기 비례 침식(두께 t)
        t = max(1, int(0.05 * min(w, h)))
        inner_mask = cv2.erode(outer_mask, cv2.getStructuringElement(cv2.MORPH_RECT, (t, t)), iterations=1)

        ring = cv2.subtract(outer_mask, inner_mask)
        ring_nonzero = cv2.countNonZero(ring)
        if ring_nonzero < 200:
            continue

        ring_mean  = cv2.mean(gray, mask=ring)[0]
        inner_mean = cv2.mean(gray, mask=inner_mask)[0]
        ring_ratio = float(ring_nonzero) / float(area)  # 고리 두께 비율

        # 고리(테두리)는 어둡고, 내부는 밝고, 두께는 충분히 두꺼움
        if (inner_mean - ring_mean) > 30 and inner_mean > 120 and 0.04 <= ring_ratio <= 0.22:
            score = (inner_mean - ring_mean) * rectangularity
            if score > best_score:
                best_score = score
                best = (x, y, w, h)

    return [best] if best else []


def find_sticker_candidates_full(image):
    """
    빨간 스티커를 강하게 추출하기 위해 엄격한 색상/형태 기준을 적용한다.
    반환: [(x,y,w,h), ...]
    """
    blurred = cv2.GaussianBlur(image, (5, 5), 0)
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

    # Hue: 0~15 (red) 또는 170~180, Saturation/Value 충분히 높음
    mask_h1 = cv2.inRange(hsv, (0, 110, 70), (15, 255, 255))
    mask_h2 = cv2.inRange(hsv, (170, 110, 70), (180, 255, 255))
    mask_hsv = cv2.bitwise_or(mask_h1, mask_h2)

    b_chan, g_chan, r_chan = cv2.split(blurred)
    r_gt_g = cv2.subtract(r_chan, g_chan)
    r_gt_b = cv2.subtract(r_chan, b_chan)
    _, mask_rg = cv2.threshold(r_gt_g, 40, 255, cv2.THRESH_BINARY)
    _, mask_rb = cv2.threshold(r_gt_b, 40, 255, cv2.THRESH_BINARY)
    _, mask_rmin = cv2.threshold(r_chan, 140, 255, cv2.THRESH_BINARY)

    mask = cv2.bitwise_and(mask_hsv, mask_rg)
    mask = cv2.bitwise_and(mask, mask_rb)
    mask = cv2.bitwise_and(mask, mask_rmin)

    lab = cv2.cvtColor(blurred, cv2.COLOR_BGR2LAB)
    _, a_chan, b_chan = cv2.split(lab)
    mask_lab = cv2.inRange(a_chan, 155, 255)
    mask_lab_b = cv2.inRange(b_chan, 120, 190)
    mask = cv2.bitwise_and(mask, mask_lab)
    mask = cv2.bitwise_and(mask, mask_lab_b)

    mask = cv2.medianBlur(mask, 5)

    close_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11))
    open_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel, iterations=1)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)), iterations=1)

    cv2.imshow("sticker_mask", mask)

    contours = _find_contours_external(mask)
    H, W = image.shape[:2]
    img_area = H * W
    candidates = []

    for c in contours:
        contour_area = cv2.contourArea(c)
        if contour_area < 0.004 * img_area or contour_area > 0.1 * img_area:
            continue
        x, y, w, h = cv2.boundingRect(c)
        if w <= 0 or h <= 0:
            continue
        rect_area = w * h
        fill_ratio = contour_area / float(rect_area)
        aspect = w / float(h)
        if fill_ratio < 0.65 or not (1.1 <= aspect <= 4.2):
            continue

        roi = image[y:y+h, x:x+w]
        if roi.size == 0:
            continue
        mean_b, mean_g, mean_r, _ = cv2.mean(roi)
        if mean_r < 150 or (mean_r - max(mean_g, mean_b)) < 35:
            continue

        candidates.append((x, y, w, h, contour_area))

    candidates.sort(key=lambda item: item[4], reverse=True)
    return [(x, y, w, h) for x, y, w, h, _ in candidates]


def find_box_roi(image):
    """
    갈색 상자를 색 기반으로 찾아 ROI(Rectangle)로 반환.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask_hsv = (
        cv2.inRange(hsv, (5, 40, 60), (18, 200, 220)) |
        cv2.inRange(hsv, (15, 20, 70), (32, 180, 210))
    ).astype(np.uint8)

    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    _, _, b_channel = cv2.split(lab)
    mask_lab = cv2.inRange(b_channel, 135, 200)

    mask = cv2.bitwise_or(mask_hsv, mask_lab)
    kernel_close = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 21))
    kernel_open = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))
    refined = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_close, iterations=1)
    refined = cv2.morphologyEx(refined, cv2.MORPH_OPEN, kernel_open, iterations=1)

    contours = _find_contours_external(refined)
    if not contours:
        return None

    H, W = image.shape[:2]
    img_area = H * W
    best_rect = None
    best_score = -1.0
    for c in contours:
        x, y, w, h = cv2.boundingRect(c)
        area = w * h
        if not (0.15 * img_area <= area <= 0.9 * img_area):
            continue
        rectangularity = cv2.contourArea(c) / float(max(area, 1))
        aspect = w / float(h + 1e-6)
        if rectangularity < 0.5 or not (0.8 <= aspect <= 2.5):
            continue
        score = rectangularity * area
        if score > best_score:
            best_score = score
            margin_x = int(0.05 * w)
            margin_y = int(0.08 * h)
        x0 = max(0, x - margin_x)
        y0 = max(0, y - margin_y)
        x1 = min(W, x + w + margin_x)
        y1 = min(H, y + h + margin_y)
        best_rect = (x0, y0, x1 - x0, y1 - y0)

    return best_rect


def detect_candidates_within_roi(frame, roi_rect):
    """
    ROI 내부에서 송장/스티커 후보를 탐색하고 전체 좌표계로 반환.
    """
    if roi_rect is None:
        return [], []
    x, y, w, h = roi_rect
    if w <= 0 or h <= 0:
        return [], []
    H, W = frame.shape[:2]
    x_end = min(W, x + w)
    y_end = min(H, y + h)
    x = max(0, x)
    y = max(0, y)
    roi_img = frame[y:y_end, x:x_end]
    if roi_img.size == 0:
        return [], []

    invoice_local = find_invoice_by_black_border(roi_img)
    sticker_local = find_sticker_candidates_full(roi_img)

    invoice_rects = [(x + rx, y + ry, rw, rh) for (rx, ry, rw, rh) in invoice_local]
    sticker_rects = [(x + sx, y + sy, sw, sh) for (sx, sy, sw, sh) in sticker_local]
    return invoice_rects, sticker_rects


def collect_stable_candidates(cap, warmup_sec=2.0, track_sec=2.0):
    """
    capture 명령 직후 2초 간 안정화하고, 이어서 track_sec 동안 후보를 추적한다.
    가장 신뢰도 높은 프레임과 후보 리스트를 반환한다.
    """
    frame_for_inference = None
    best_invoice = []
    best_stickers = []
    best_box_roi = None
    best_invoice_score = -1.0
    best_sticker_score = -1.0
    last_roi = None

    end_warmup = time.time() + max(0.0, warmup_sec)
    while time.time() < end_warmup:
        ret, frame = cap.read()
        if not ret:
            break
        frame_for_inference = frame
        cv2.waitKey(1)
        time.sleep(0.01)

    end_track = time.time() + max(0.0, track_sec)
    while time.time() < end_track:
        ret, frame = cap.read()
        if not ret:
            break
        frame_for_inference = frame
        box_roi = find_box_roi(frame)
        if box_roi:
            last_roi = box_roi
        elif last_roi is None:
            cv2.waitKey(1)
            time.sleep(0.01)
            continue

        active_roi = box_roi if box_roi else last_roi
        invoice_rects, sticker_rects = detect_candidates_within_roi(frame, active_roi)

        if invoice_rects:
            rect = max(invoice_rects, key=lambda r: r[2] * r[3])
            score = rect[2] * rect[3]
            if score > best_invoice_score:
                best_invoice_score = score
                best_invoice = [rect]
                best_box_roi = active_roi

        if sticker_rects:
            total_area = sum(r[2] * r[3] for r in sticker_rects)
            score = len(sticker_rects) * 1000 + total_area
            if score > best_sticker_score:
                best_sticker_score = score
                best_stickers = sticker_rects
                best_box_roi = active_roi

        cv2.waitKey(1)
        time.sleep(0.01)

    if best_box_roi is None:
        best_box_roi = last_roi
    return frame_for_inference, best_box_roi, best_invoice, best_stickers


# --- OCR 관련 함수 (기존 유지) ---
def perform_robust_ocr(image, dictionary):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.convertScaleAbs(gray, alpha=1.3, beta=10)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    clahe_img = clahe.apply(enhanced)

    resized = cv2.resize(clahe_img, None, fx=1.5, fy=1.5, interpolation=cv2.INTER_LINEAR)

    _, thresh1 = cv2.threshold(resized, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    thresh1_inv = cv2.bitwise_not(thresh1)
    padded1 = cv2.copyMakeBorder(thresh1_inv, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)

    thresh2 = cv2.adaptiveThreshold(resized, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 5)
    padded2 = cv2.copyMakeBorder(thresh2, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)

    thresh3 = cv2.adaptiveThreshold(resized, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, 15, 3)
    padded3 = cv2.copyMakeBorder(thresh3, 10, 10, 10, 10, cv2.BORDER_CONSTANT, value=0)

    kernel = np.ones((3,3), np.uint8)
    dilated = cv2.dilate(padded1, kernel, iterations=1)
    eroded = cv2.erode(padded1, kernel, iterations=1)

    cv2.imshow("ocr_gray_resized", resized)
    cv2.imshow("ocr_thresh1_inv", padded1)
    cv2.imshow("ocr_thresh2", padded2)
    cv2.imshow("ocr_thresh3", padded3)
    cv2.imshow("ocr_dilated", dilated)
    cv2.imshow("ocr_eroded", eroded)

    attempts = [
        (padded1, r'--oem 1 --psm 7'),
        (padded1, r'--oem 1 --psm 8'),
        (dilated, r'--oem 1 --psm 8'),
        (eroded, r'--oem 1 --psm 8'),
        (padded2, r'--oem 1 --psm 8'),
        (padded3, r'--oem 1 --psm 7'),
    ]
    for i, (img, config) in enumerate(attempts):
        raw_text = pytesseract.image_to_string(img, lang='kor', config=config).strip()
        cleaned_text = re.sub('[^가-힣]', '', raw_text)
        if cleaned_text:
            print(f"   [OCR Attempt #{i+1}] Raw: '{raw_text}', Cleaned: '{cleaned_text}'")
        for word in dictionary:
            if word in cleaned_text:
                print(f"   ===> SUCCESS on attempt #{i+1}!")
                return word
    return None


# --- 메인 실행 블록 ---
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Sticker/Invoice detector with TCP control (no Box ROI).")
    parser.add_argument("--server-host", default=TCP_HOST, help="TCP 서버 IP 주소")
    parser.add_argument("--server-port", type=int, default=TCP_PORT, help="TCP 서버 포트")
    parser.add_argument("--client-name", default=TCP_CLIENT_NAME, help="이 장치의 ID")
    parser.add_argument("--tcp-target", default="qt", help="서버로 메시지를 보낼 기본 대상 ID")
    args, unknown_args = parser.parse_known_args()
    if unknown_args:
        print(f"Warning: Unknown arguments ignored: {unknown_args}")

    yolo_detector = YOLOv8_Detector(ONNX_MODEL_PATH, CONF_THRESHOLD, IOU_THRESHOLD)

    capture_client = CaptureCommandClient(
        args.server_host,
        args.server_port,
        client_name=args.client_name,
        default_target=args.tcp_target,
    )
    if not capture_client.is_connected():
        print("Warning: TCP capture client is not connected yet. Retrying in background.")

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"Error: {CAMERA_INDEX} camera not opened")
        exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Program Start : press 'q' to quit")

    last_detection_time = 0
    last_box_roi = None
    last_invoice_candidates = []
    last_sticker_candidates = []

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        current_time = time.time()

        # 주기적으로 전체 프레임에서 바로 후보 탐색 (Box ROI 단계 제거)
        if (current_time - last_detection_time) > DETECTION_INTERVAL:
            last_detection_time = current_time

            box_roi = find_box_roi(frame)
            if box_roi:
                last_box_roi = box_roi
                invoice_rects, sticker_rects = detect_candidates_within_roi(frame, box_roi)
            elif last_box_roi:
                invoice_rects, sticker_rects = detect_candidates_within_roi(frame, last_box_roi)
            else:
                invoice_rects, sticker_rects = [], []

            last_invoice_candidates = invoice_rects
            last_sticker_candidates = sticker_rects

        # 시각화
        display_frame = frame.copy()
        if last_box_roi:
            bx, by, bw, bh = last_box_roi
            cv2.rectangle(display_frame, (bx, by), (bx + bw, by + bh), (255, 255, 255), 2)
            cv2.putText(display_frame, "Box_ROI", (bx, by - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        for rect in last_invoice_candidates:
            x, y, w, h = rect
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(display_frame, "Invoice_Candidate", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        for rect in last_sticker_candidates:
            x, y, w, h = rect
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(display_frame, "Sticker_Candidate", (x, y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)
        cv2.imshow("Real-time Candidate Detection", display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('q'):
            break

        # --- TCP/IP 'capture' 명령 처리 ---
        if capture_client and capture_client.check_and_clear_capture():
            print("\n--- 'capture' command received, start inference ---")

            stabilized_frame, tracked_box_roi, tracked_invoice, tracked_stickers = collect_stable_candidates(
                cap, warmup_sec=2.0, track_sec=2.0
            )
            if stabilized_frame is not None:
                frame_for_inference = stabilized_frame
            else:
                frame_for_inference = frame

            if tracked_box_roi:
                last_box_roi = tracked_box_roi
            if tracked_invoice:
                last_invoice_candidates = tracked_invoice
            if tracked_stickers:
                last_sticker_candidates = tracked_stickers

            result_frame = frame_for_inference.copy()
            ocr_text_final = None

            # 송장 OCR
            if not last_invoice_candidates:
                print("Invoice candidate not detected.")
            else:
                rect = max(last_invoice_candidates, key=lambda r: r[2] * r[3])
                x, y, w, h = rect
                roi = frame_for_inference[y:y+h, x:x+w]
                ocr_text_final = perform_robust_ocr(roi, OCR_DICTIONARY)
                eng_map = {"서울": "Seoul", "대전": "Daejeon", "부산": "Busan"}
                display_text = eng_map.get(ocr_text_final, "FAIL")
                cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(result_frame, f"OCR: {display_text}", (x, y - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # 스티커 분류 (YOLO)
            det_classes = []
            if not last_sticker_candidates:
                print("Sticker candidate not detected.")
            for i, rect in enumerate(last_sticker_candidates):
                if len(rect) == 4:
                    x, y, w, h = rect
                    roi = frame_for_inference[y:y+h, x:x+w]
                    if roi.size == 0:
                        continue
                    print(f"--- Sticker Candidate #{i+1} at ({x}, {y}) ---")
                    boxes, scores, class_ids = yolo_detector.detect_objects(roi)
                    if len(boxes) > 0:
                        print(f"   ...Sticker candidate -> AI detected {len(boxes)} object(s).")
                        for box, score, class_id in zip(boxes, scores, class_ids):
                            class_name = YOLO_CLASS_NAMES[class_id]
                            print(f"   ===> RESULT: Detected '{class_name}' (Confidence: {score:.2f})")
                            det_classes.append(class_name)
                            x1_rel, y1_rel, x2_rel, y2_rel = box.astype(int)
                            x1_abs, y1_abs = x + x1_rel, y + y1_rel
                            x2_abs, y2_abs = x + x2_rel, y + y2_rel
                            label = f"{class_name}: {score:.2f}"
                            cv2.rectangle(result_frame, (x1_abs, y1_abs), (x2_abs, y2_abs), (0, 0, 255), 2)
                            cv2.putText(result_frame, label, (x1_abs, y1_abs - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        print(f"   ...Sticker candidate at ({x}, {y}) -> AI detected nothing.")

            unique_det_classes = []
            if det_classes:
                unique_det_classes = sorted(list(set(det_classes)))
                print(f"Original detections: {det_classes}")
                print(f"Unique detections: {unique_det_classes}")

            # TCP 전송
            print("Sending encoded inference results to server...")
            send_encoded_results(capture_client, ocr_text_final, unique_det_classes, target="qt")

            print("Sending original image to server...")
            CaptureCommandClient.send_image(capture_client, frame_for_inference, target="qt")

            cv2.imshow("Inference Result", result_frame)
            print("------------------------------------")

    print("closing program...")
    if capture_client:
        capture_client.shutdown()
    cap.release()
    cv2.destroyAllWindows()

