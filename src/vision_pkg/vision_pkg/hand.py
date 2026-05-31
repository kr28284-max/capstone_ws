import cv2
import time
from ultralytics import YOLO
import mediapipe as mp
from collections import deque, Counter

model = YOLO(
    r"C:\Users\user\Desktop\ì¸ì²œëŒ€\2026_1 ê°•ì˜\ìº¡ìŠ¤í†¤ì„¤ê³„1\realsense_project\best.pt"
)

class_map = {
    1: "bearing",
    2: "boltnut",
    3: "gear",
    4: "damper"
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

def count_fingers(lm, hand_label):

    fingers = []



    if hand_label == "Right":

        # flip ì ìš© í›„ ë°˜ì „
        fingers.append(1 if lm[4][0] < lm[3][0] else 0)

    else:

        # flip ì ìš© í›„ ë°˜ì „
        fingers.append(1 if lm[4][0] > lm[3][0] else 0)


    tips = [8, 12, 16, 20]
    pips = [6, 10, 14, 18]

    for tip, pip in zip(tips, pips):

        fingers.append(1 if lm[tip][1] < lm[pip][1] else 0)

    return sum(fingers)

cap = cv2.VideoCapture(0)

cap.set(3, 640)
cap.set(4, 480)

mode = "HAND"

selected_class = None

# ì†ê°€ë½ ì•ˆì •í™”
scan_history = deque(maxlen=30)
scan_start_time = None
SCAN_DURATION = 2.0

# YOLO
frame_count = 0
YOLO_INTERVAL = 3
last_results = None


object_start_time = None
OBJECT_DURATION = 4.0


mediapipe_pause_until = 0


HAND_CLOSE_MIN_SIZE = 120


hand_return_start = None
HAND_RETURN_DURATION = 1.5

while True:

    ret, frame = cap.read()

    if not ret:
        break


    frame = cv2.flip(frame, 1)

    frame_count += 1
    current_time = time.time()

    mediapipe_enabled = current_time > mediapipe_pause_until

    finger_count = None
    hand_detected = False

    if mediapipe_enabled:

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        result = hands.process(rgb)

        if result.multi_hand_landmarks:

            hand_detected = True

            for idx, hand_landmarks in enumerate(result.multi_hand_landmarks):

                h, w, _ = frame.shape

                lm = [
                    (int(p.x * w), int(p.y * h))
                    for p in hand_landmarks.landmark
                ]

                
                xs = [pt[0] for pt in lm]
                ys = [pt[1] for pt in lm]
                hand_size = max(max(xs) - min(xs), max(ys) - min(ys))
                if hand_size < HAND_CLOSE_MIN_SIZE:
                    cv2.putText(
                        frame,
                        "HAND TOO FAR",
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 0, 255),
                        2
                    )
                    continue

                
                hand_label = (
                    result.multi_handedness[idx]
                    .classification[0]
                    .label
                )

                
                finger_count = count_fingers(lm, hand_label)

                
                mp_draw.draw_landmarks(
                    frame,
                    hand_landmarks,
                    mp_hands.HAND_CONNECTIONS
                )

                
                cv2.putText(
                    frame,
                    f"{hand_label} Hand",
                    (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 0),
                    2
                )

    # ==================================================
    # HAND MODE
    # ==================================================
    if mode == "HAND":

        if finger_count in class_map:

            if scan_start_time is None:
                scan_start_time = current_time

            scan_history.append(finger_count)


            if current_time - scan_start_time > SCAN_DURATION:

                most_common = Counter(scan_history).most_common(1)[0][0]

                selected_class = class_map[most_common]

                
                mode = "OBJECT"

                object_start_time = current_time

               
                mediapipe_pause_until = current_time + 2.0

                scan_history.clear()
                scan_start_time = None

        else:
            scan_start_time = None
            scan_history.clear()

        # ---------------------------
        # UI
        # ---------------------------
        if finger_count is not None:

            cv2.putText(
                frame,
                f"Fingers: {finger_count}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2
            )

        if scan_start_time:

            cv2.putText(
                frame,
                "SCANNING...",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

    # ==================================================
    # OBJECT MODE
    # ==================================================
    elif mode == "OBJECT":

        
        if mediapipe_enabled and hand_detected:

            
            if hand_return_start is None:
                hand_return_start = current_time

            
            if current_time - hand_return_start > HAND_RETURN_DURATION:

                mode = "HAND"

                selected_class = None
                last_results = None

                scan_history.clear()
                scan_start_time = None

                hand_return_start = None

                cv2.putText(
                    frame,
                    "RETURN TO HAND MODE",
                    (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 0, 255),
                    2
                )

                cv2.imshow("Final System", frame)
                cv2.waitKey(1)

                continue

        
        else:
            hand_return_start = None

       
        if frame_count % YOLO_INTERVAL == 0:

            last_results = model(
                frame,
                imgsz=320,
                conf=0.4,
                verbose=False
            )[0]

        
        if last_results and selected_class:

            for box in last_results.boxes:

                cls_id = int(box.cls[0])
                name = model.names[cls_id]

                
                if name != selected_class:
                    continue

                x1, y1, x2, y2 = map(int, box.xyxy[0])

                
                cv2.rectangle(
                    frame,
                    (x1, y1),
                    (x2, y2),
                    (0, 255, 0),
                    2
                )

                
                cv2.putText(
                    frame,
                    name,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 0),
                    2
                )

        # ----------------------------------------------
        # UI
        # ----------------------------------------------
        cv2.putText(
            frame,
            f"TARGET: {selected_class}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 0),
            2
        )

        # MediaPipe OFF 
        if not mediapipe_enabled:

            cv2.putText(
                frame,
                "Waiting",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 165, 255),
                2
            )

      
        else:

            cv2.putText(
                frame,
                "SHOW HAND TO RETURN",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2
            )

        
        if current_time - object_start_time > OBJECT_DURATION:

            mode = "HAND"

            selected_class = None
            last_results = None

            scan_history.clear()
            scan_start_time = None

            hand_return_start = None

    
    cv2.imshow("Final System", frame)

    # ESC ì¢…ë£Œ
    if cv2.waitKey(1) == 27:
        break


cap.release()
cv2.destroyAllWindows()