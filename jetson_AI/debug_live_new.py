import cv2
import numpy as np
import os
import argparse
import pytesseract
import onnxruntime as ort
from Levenshtein import distance
import time

# --- ⭐ 설정 부분 ⭐ ---
# YOLOv8 ONNX 모델 및 관련 설정
ONNX_MODEL_PATH = "best.onnx" 
YOLO_CLASS_NAMES = ['care', 'fragile', 'water', 'upside'] 

# OCR 관련 설정
OCR_DICTIONARY = ["서울", "부산", "대전"]

# 카메라 및 탐지 설정
CAMERA_INDEX = 2
CONF_THRESHOLD = 0.4 # AI가 스티커를 판단하는 확신도 임계값
IOU_THRESHOLD = 0.5
DETECTION_INTERVAL = 0.5
# --- ⭐ 설정 끝 ⭐ ---


# --- 1. YOLOv8 탐지기 클래스 ---
class YOLOv8_Detector:
    def __init__(self, model_path, confidence_thres=0.5, iou_thres=0.5):
        self.confidence_thres, self.iou_thres = confidence_thres, iou_thres
        try:
            self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider'])
            #self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
            #self.session = ort.InferenceSession(model_path, providers=[ 'CPUExecutionProvider'])
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

        # --- ✨ 시간 측정 로직 추가 ✨ ---
        start_time = time.time()
        outputs = self.session.run(self.output_names, {self.input_names[0]: input_tensor})
        end_time = time.time()
        
        inference_time = (end_time - start_time) * 1000 # 초(s)를 밀리초(ms)로 변환
        print(f"   ===> Inference Time: {inference_time:.2f} ms")
        # ------------------------------------

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
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), scores.tolist(), self.confidence_thres, self.iou_thres)
        if len(indices) > 0:
            indices = indices.flatten()
            return boxes[indices], scores[indices], class_ids[indices]
        else:
            return [], [], []

def find_box_candidate(image):
    """ ✨ 새로 추가된 함수 ✨
    이미지에서 가장 큰 윤곽선(박스)을 찾아 좌표를 반환합니다. """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY_INV)
    
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if not contours:
        return None
    
    # 면적이 가장 큰 윤곽선을 박스로 간주
    largest_contour = max(contours, key=cv2.contourArea)
    
    # 박스가 프레임의 95% 이상을 차지하면 무시 (전체 화면이 잡히는 것 방지)
    image_area = image.shape[0] * image.shape[1]
    if cv2.contourArea(largest_contour) > image_area * 0.95:
        return None
        
    return cv2.boundingRect(largest_contour)

# --- 2. OpenCV 후보 탐색 함수들 ---
def find_invoice_candidates(image):
    """ ✨ 더욱 강력해진 최종 필터링 함수 ✨ """
    candidates = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    threshold_value = 60 
    _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
    cv2.imshow("Invoice Threshold Debug", thresh)
    
    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 1. 탐지된 모든 윤곽선을 하나씩 검사합니다.
    for c in contours:
        area = cv2.contourArea(c)
        
        # 2. 면적 조건을 통과하는지 확인합니다. (최댓값 300000 유지)
        if 2000 < area < 300000:
            rect = cv2.boundingRect(c)
            x, y, w, h = rect
            aspect_ratio = w / float(h) if h > 0 else 0
            
            # 3. 사각형에 가까운 비율인지 확인합니다. (가장 중요한 필터!)
            # 이 필터가 빛 반사 같은 불규칙한 모양을 걸러줍니다.
            if 1.0 < aspect_ratio < 5.0:
                candidates.append(rect)
    
    # 4. 조건을 통과한 후보들 중, 가장 면적이 넓은 것을 최종 선택합니다.
    if candidates:
        largest_candidate = max(candidates, key=lambda r: r[2] * r[3])
        return [largest_candidate]
            
    return []

### debug find invoice
#def find_invoice_candidates(image):
#    """ ✨ 디버깅 기능이 대폭 강화된 함수 ✨ """
#    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#    
#    # 이전에 설정하신 50을 그대로 사용합니다.
#    threshold_value = 50 
#    _, thresh = cv2.threshold(gray, threshold_value, 255, cv2.THRESH_BINARY_INV)
#    cv2.imshow("Invoice Threshold Debug", thresh)
#    
#    contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#    
#    # --- ⭐ 디버깅 출력 시작 ⭐ ---
#    print(f"  [1] Found {len(contours)} contours.")
#
#    if not contours:
#        print("  [Final] No valid invoice candidate found in this frame.")
#        return []
#
#    largest_contour = max(contours, key=cv2.contourArea)
#    area = cv2.contourArea(largest_contour)
#    print(f"  [2] Largest contour area: {area:.2f}")
#
#    # 크기 필터: 이 범위가 문제일 수 있습니다.
#    if 2000 < area < 100000:
#        print("  [3] Area is within the valid range.")
#        rect = cv2.boundingRect(largest_contour)
#        x, y, w, h = rect
#        aspect_ratio = w / float(h) if h > 0 else 0
#        print(f"  [4] Aspect ratio: {aspect_ratio:.2f}")
#        
#        # 비율 필터: 현재 주석 처리된 상태로 둡니다.
#        # if 1.0 < aspect_ratio < 5.0:
#        print("  [5] Candidate found! Returning rectangle.")
#        return [rect]
#            
#    print("  [Final] No valid invoice candidate found in this frame.")
#    return []
#    # --- ⭐ 디버깅 출력 끝 ⭐ ---

def find_sticker_candidates(image):
    """ ✨ 새로 추가된 함수 ✨
    OpenCV로 스티커 후보(빨간색 객체)의 '좌표' 리스트를 반환합니다."""
    candidates = []
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    
    # 빨간색의 HSV 범위 (두 부분으로 나누어 처리)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])
    
    # 마스크 생성 및 결합
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
    mask = cv2.add(mask1, mask2)
    
    # --- ⭐ 추가된 부분 시작 ⭐ ---
    # 닫힘(Closing) 연산을 위한 커널 생성
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (15, 15))
    # 닫힘 연산을 적용하여 끊어진 부분을 이어붙임
    closed_mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    # --- ⭐ 추가된 부분 끝 ⭐ ---

    contours, _ = cv2.findContours(closed_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    for c in contours:
        area = cv2.contourArea(c)
        if area > 500: # 너무 작은 객체는 노이즈로 간주하고 무시
            candidates.append(cv2.boundingRect(c))
            
    return candidates


# --- 3. OCR 관련 함수들 ---
def correct_spelling(text, dictionary, max_distance=1):
    if not text: return "OCR_FAIL"
    if text in dictionary: return text
    closest_word, min_dist = min([(word, distance(text, word)) for word in dictionary], key=lambda x: x[1])
    if min_dist <= max_distance:
        return closest_word
    return text

def perform_ocr(image, dictionary):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    _, thresh = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    raw_text = pytesseract.image_to_string(thresh, lang='kor', config=r'--oem 1 --psm 7').strip()
    return correct_spelling(raw_text, dictionary)


# --- ✨ 수정된 메인 실행 블록 ---
if __name__ == '__main__':
    yolo_detector = YOLOv8_Detector(ONNX_MODEL_PATH, CONF_THRESHOLD, IOU_THRESHOLD)

    cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened(): print(f"Error: {CAMERA_INDEX} camera not opened"); exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Operation Start : press 's' to inference, press 'q' to quit")
    
    last_detection_time = 0
    last_box_candidate = None
    last_invoice_candidates = []
    last_sticker_candidates = []

    while True:
        ret, frame = cap.read()
        if not ret: break

        current_time = time.time()
        
        if (current_time - last_detection_time) > DETECTION_INTERVAL:
            last_detection_time = current_time
            
            # 1. 프레임 전체에서 박스 후보를 먼저 찾습니다.
            box_rect = find_box_candidate(frame)
            last_box_candidate = box_rect
            
            # 박스를 찾았을 경우에만 내부 탐색
            if box_rect:
                x_box, y_box, w_box, h_box = box_rect
                # 박스 영역만 잘라냅니다 (ROI)
                box_roi = frame[y_box:y_box+h_box, x_box:x_box+w_box]
                # 1. 박스 영역 안에서 가능한 모든 송장 후보를 찾습니다.
                all_invoice_rects = find_invoice_candidates(box_roi)
                sticker_rects_relative = find_sticker_candidates(box_roi)

                # ✨ 2. 찾은 송장 후보들 중 면적이 가장 큰 것 하나만 선택합니다. ✨
                if all_invoice_rects:
                    # 면적(w*h)을 기준으로 가장 큰 사각형을 찾습니다.
                    largest_invoice_rect = max(all_invoice_rects, key=lambda r: r[2] * r[3])
                    invoice_rects_relative = [largest_invoice_rect] # 리스트 형태로 저장
                else:
                    invoice_rects_relative = []

                # 2. 박스 영역 안에서 송장과 스티커를 찾습니다.
                # 찾은 좌표는 box_roi 기준 상대 좌표입니다.
                invoice_rects_relative = find_invoice_candidates(box_roi)
                sticker_rects_relative = find_sticker_candidates(box_roi)
                
                # 3. 찾은 좌표를 전체 프레임 기준으로 변환합니다.
                last_invoice_candidates = [(r[0] + x_box, r[1] + y_box, r[2], r[3]) for r in invoice_rects_relative]
                last_sticker_candidates = [(s[0] + x_box, s[1] + y_box, s[2], s[3]) for s in sticker_rects_relative]
            else:
                # 박스를 못 찾으면 모든 후보를 초기화합니다.
                last_invoice_candidates = []
                last_sticker_candidates = []

        # 실시간 디버깅 화면 표시
        display_frame = frame.copy()
        if last_box_candidate: # 박스 후보 (흰색)
            x, y, w, h = last_box_candidate
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 255, 255), 3)
            cv2.putText(display_frame, "Box_ROI", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)

        for rect in last_invoice_candidates: # 송장 후보 (녹색)
            x, y, w, h = rect
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.putText(display_frame, "Invoice_Candidate", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        for rect in last_sticker_candidates: # 스티커 후보 (파란색)
            x, y, w, h = rect
            cv2.rectangle(display_frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.putText(display_frame, "Sticker_Candidate", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 0), 2)

        cv2.imshow("Real-time Candidate Detection", display_frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            # (이하 's'키를 눌렀을 때의 로직은 변경 없음)
            print("\n--- 's' pressed, start inference ---")
            result_frame = frame.copy()
            if not last_invoice_candidates: print("Invoice candidate not detected by OpenCV.")
            for rect in last_invoice_candidates:
                if len(rect) == 4:
                     x, y, w, h = rect
                     roi = frame[y:y+h, x:x+w]
                     ocr_text = perform_ocr(roi, OCR_DICTIONARY)
                     print(f"Invoice candidate -> OCR result: {ocr_text}")
                     cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                     cv2.putText(result_frame, f"OCR: {ocr_text}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            if not last_sticker_candidates: print("Sticker candidate not detected by OpenCV.")
            for i, rect in enumerate(last_sticker_candidates):
                if len(rect) == 4:
                    x, y, w, h = rect
                    roi = frame[y:y+h, x:x+w]
                    if roi.size == 0: continue

                    # 스티커 후보별로 번호를 붙여서 출력
                    print(f"--- Sticker Candidate #{i+1} at ({x}, {y}) ---")

                    boxes, scores, class_ids = yolo_detector.detect_objects(roi)

                    if len(boxes) > 0:
                        print(f"   ...Sticker candidate at ({x}, {y}) -> AI detected {len(boxes)} object(s).")
                        for box, score, class_id in zip(boxes, scores, class_ids):
                            # --- ⭐ 추가된 부분 시작 ⭐ ---
                            # 인식된 객체의 클래스 이름과 신뢰도를 터미널에 출력
                            class_name = YOLO_CLASS_NAMES[class_id]
                            print(f"   ===> RESULT: Detected '{class_name}' (Confidence: {score:.2f})")
                            # --- ⭐ 추가된 부분 끝 ⭐ ---
                            
                            # (화면에 그리는 부분은 변경 없음)
                            x1_rel, y1_rel, x2_rel, y2_rel = box.astype(int)
                            # 버그 수정: x_box, y_box가 아닌 현재 스티커 후보의 x, y를 더해야 함
                            x1_abs, y1_abs = x + x1_rel, y + y1_rel
                            x2_abs, y2_abs = x + x2_rel, y + y2_rel
                            label = f"{class_name}: {score:.2f}"
                            cv2.rectangle(result_frame, (x1_abs, y1_abs), (x2_abs, y2_abs), (0, 0, 255), 2)
                            cv2.putText(result_frame, label, (x1_abs, y1_abs - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                    else:
                        print(f"   ...Sticker candidate at ({x}, {y}) -> AI detected nothing.")
            cv2.imshow("Inference Result", result_frame)
            print("------------------------------------")

        elif key == ord('q'):
            break

    print("closing program...")
    cap.release()
    cv2.destroyAllWindows()
