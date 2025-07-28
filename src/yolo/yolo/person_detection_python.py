# from ultralytics import YOLO
# import cv2
# import time

# # ---------- parameter ----------
# model_path = ".pt"        # module
# camera_id = 0                # The default for USB cameras is /dev/video0
# conf_threshold = 0.5          # confidence
# class_name_to_detect = 'person' 

# # ---------- import module ----------
# model = YOLO(model_path)

# # ---------- open camera ----------
# cap = cv2.VideoCapture(camera_id)
# if not cap.isOpened():
#     print("cannot open camera")
#     exit()

# # ---------- inference main loop ----------
# prev_time = time.time()

# while True:
#     ret, frame = cap.read()
#     if not ret:
#         print("cannot read image")
#         break

#     # inference
#     results = model(frame, imgsz=320)[0]

#     # draw the frame line
#     annotated_frame = results.plot()


#     # calculate FPS
#     curr_time = time.time()
#     fps = 1 / (curr_time - prev_time)
#     prev_time = curr_time
#     cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
#                 cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

#     # show the result
#     cv2.imshow("YOLO Detection", annotated_frame)

#     # press q to quit
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         break

# # ---------- end of cleanup ----------
# cap.release()
# cv2.destroyAllWindows()
from ultralytics import YOLO
import cv2
import time

# ---------- parameter ----------
model_path = "best.pt"    # 替換成實際模型檔案
camera_id = 0
conf_threshold = 0.5
class_name_to_detect = 'person'

# ---------- import module ----------
model = YOLO(model_path)
class_names = model.names  # 獲取所有類別名稱

# ---------- open camera ----------
cap = cv2.VideoCapture(camera_id)
if not cap.isOpened():
    print("cannot open camera")
    exit()

# ---------- inference main loop ----------
prev_time = time.time()

while True:
    ret, frame = cap.read()
    if not ret:
        print("cannot read image")
        break

    # inference
    results = model(frame, imgsz=320)[0]

    # 建立繪圖 frame 的複本
    annotated_frame = frame.copy()

    for box in results.boxes:
        cls_id = int(box.cls[0])
        conf = float(box.conf[0])
        if conf >= conf_threshold and class_names[cls_id] == class_name_to_detect:
            # 取得框座標
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            label = f'{class_names[cls_id]} {conf:.2f}'
            # 畫框
            cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            # 顯示文字
            cv2.putText(annotated_frame, label, (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 計算 FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time)
    prev_time = curr_time
    cv2.putText(annotated_frame, f"FPS: {fps:.2f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # 顯示畫面
    cv2.imshow("YOLO Detection", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ---------- end of cleanup ----------
cap.release()
cv2.destroyAllWindows()
