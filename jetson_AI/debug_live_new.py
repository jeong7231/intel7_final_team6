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
            self.session = ort.InferenceSession(model_path, providers=['CUDAExecutionProvider', 'CPUExecutionProvider'])
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
        outputs = self.session.run(self.output_names, {self.input_names[0]: input_tensor})
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


# --- 2. OpenCV 후보 탐색 함수들 ---
def find_invoice_candidates(image):
    """OpenCV로 송장 후보(흰색 사각형)의 '좌표' 리스트를 반환합니다."""
    candidates = []
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (7, 7), 0)
    _, thresh = cv2.threshold(blurred, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    for c in contours:
        area = cv2.contourArea(c)
        if area > 4000:
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) == 4:
                rect = cv2.boundingRect(c)
                x, y, w, h = rect
                aspect_ratio = w / float(h) if h > 0 else 0
                if 1.0 < aspect_ratio < 4.0:
                    candidates.append(rect)
    return candidates

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
    
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
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
    if not cap.isOpened():
        print(f"Error: {CAMERA_INDEX} camera not opened"); exit()
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("Operation Start : press 's' to inference, press 'q' to quit")
    
    last_detection_time = 0
    last_invoice_candidates = []
    last_sticker_candidates = [] # YOLO 결과 대신 OpenCV 후보를 저장

    while True:
        ret, frame = cap.read()
        if not ret: break

        current_time = time.time()
        
        if (current_time - last_detection_time) > DETECTION_INTERVAL:
            last_detection_time = current_time
            # ✨ 이제 OpenCV가 송장과 스티커 후보를 모두 찾습니다.
            last_invoice_candidates = find_invoice_candidates(frame)
            last_sticker_candidates = find_sticker_candidates(frame)

        # 실시간 디버깅 화면에 후보들을 표시
        display_frame = frame.copy()
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
            print("\n--- 's' pressed, start inference ---")
            result_frame = frame.copy()

            # 1. 송장 후보 OCR 실행
            if not last_invoice_candidates: print("Invoice candidate not detected by OpenCV.")
            for rect in last_invoice_candidates:
                x, y, w, h = rect
                roi = frame[y:y+h, x:x+w]
                ocr_text = perform_ocr(roi, OCR_DICTIONARY)
                print(f"Invoice candidate -> OCR result: {ocr_text}")
                cv2.rectangle(result_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                cv2.putText(result_frame, f"OCR: {ocr_text}", (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            # ✨ 2. 스티커 후보 영역을 잘라내어 YOLO로 추론 실행
            if not last_sticker_candidates: print("Sticker candidate not detected by OpenCV.")
            for rect in last_sticker_candidates:
                x, y, w, h = rect
                # OpenCV가 찾은 후보 영역을 잘라냄 (ROI)
                roi = frame[y:y+h, x:x+w]
                if roi.size == 0: continue

                # 잘라낸 이미지(ROI)로 YOLO 추론 수행
                boxes, scores, class_ids = yolo_detector.detect_objects(roi)
                
                if len(boxes) > 0:
                    print(f"   ...Sticker candidate at ({x}, {y}) -> AI detected {len(boxes)} object(s).")
                    # ROI 안에서 찾은 결과를 전체 화면 좌표로 변환하여 표시
                    for box, score, class_id in zip(boxes, scores, class_ids):
                        x1_rel, y1_rel, x2_rel, y2_rel = box.astype(int)
                        
                        # 전체 프레임 기준 좌표로 변환
                        x1_abs, y1_abs = x + x1_rel, y + y1_rel
                        x2_abs, y2_abs = x + x2_rel, y + y2_rel
                        
                        label = f"{YOLO_CLASS_NAMES[class_id]}: {score:.2f}"
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
