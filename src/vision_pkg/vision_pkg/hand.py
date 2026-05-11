import cv2
import time
import numpy as np
from ultralytics import YOLO
import mediapipe as mp
from collections import deque, Counter

# ---------------------------
# 모델
# ---------------------------
model = YOLO("/home/user/capstone_ws/best.pt")

# ---------------------------
# 클래스 매핑
# ---------------------------
class_map = {
    1: "bearing",
    2: "boltnut",
    3: "gear",
    4: "wheel"
}

# ---------------------------
# MediaPipe
# ---------------------------
mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

# ---------------------------
# 손가락 함수
# ---------------------------
def count_fingers(lm):
    fingers = []
    fingers.append(1 if lm[4][0] > lm[3][0] else 0)

    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]

    for tip, pip in zip(tips, pips):
        fingers.append(1 if lm[tip][1] < lm[pip][1] else 0)

    return sum(fingers)

# ---------------------------
# 카메라 설정 (RGB 강제 추가 🔥)
# ---------------------------
cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)  # ⭐ 추가

# ⭐ 추가: MJPG 포맷 강제 → 컬러 보장
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 기존 해상도 설정
cap.set(3, 640)
cap.set(4, 480)

# ⭐ 추가: 디버깅 (컬러 확인)
ret, test_frame = cap.read()
if ret:
    print("Frame shape:", test_frame.shape)
    if len(test_frame.shape) == 2:
        print("⚠️ 현재 카메라는 흑백(그레이)로 잡히고 있음")
    else:
        print("✅ 컬러(RGB)로 정상 동작 중")

# ---------------------------
# 상태 변수
# ---------------------------
mode = "HAND"  # HAND / OBJECT

selected_class = None

# 손가락 안정화
scan_history = deque(maxlen=30)
scan_start_time = None
SCAN_DURATION = 3.0

# YOLO
frame_count = 0
YOLO_INTERVAL = 3
last_results = None

# OBJECT 유지 시간
object_start_time = None
OBJECT_DURATION = 8.0

# ---------------------------
# 메인 루프
# ---------------------------
while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1
    current_time = time.time()

    # ===========================
    # HAND 모드
    # ===========================
    if mode == "HAND":
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = hands.process(rgb)

        finger_count = None

        if result.multi_hand_landmarks:
            for hand_landmarks in result.multi_hand_landmarks:
                h, w, _ = frame.shape
                lm = [(int(p.x*w), int(p.y*h)) for p in hand_landmarks.landmark]

                finger_count = count_fingers(lm)

                mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

        # 손가락 안정화
        if finger_count in class_map:
            if scan_start_time is None:
                scan_start_time = current_time

            scan_history.append(finger_count)

            if current_time - scan_start_time > SCAN_DURATION:
                most_common = Counter(scan_history).most_common(1)[0][0]

                selected_class = class_map[most_common]

                mode = "OBJECT"
                object_start_time = current_time

                scan_history.clear()
                scan_start_time = None
        else:
            scan_start_time = None
            scan_history.clear()

        # UI
        if finger_count is not None:
            cv2.putText(frame, f"Fingers: {finger_count}", (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

        if scan_start_time:
            cv2.putText(frame, "SCANNING...", (10,60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)

    # ===========================
    # OBJECT 모드
    # ===========================
    elif mode == "OBJECT":

        if frame_count % YOLO_INTERVAL == 0:
            last_results = model(frame, imgsz=320, conf=0.4, verbose=False)[0]

        if last_results and selected_class:
            for box in last_results.boxes:
                cls_id = int(box.cls[0])
                name = model.names[cls_id]

                if name != selected_class:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                cv2.rectangle(frame, (x1,y1), (x2,y2), (0,255,0), 2)
                cv2.putText(frame, name, (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.putText(frame, f"TARGET: {selected_class}", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,0,0), 2)

        if current_time - object_start_time > OBJECT_DURATION:
            mode = "HAND"
            selected_class = None
            last_results = None

    # ---------------------------
    cv2.imshow("Final System", frame)

    if cv2.waitKey(1) == 27:
        break

cap.release()
cv2.destroyAllWindows()