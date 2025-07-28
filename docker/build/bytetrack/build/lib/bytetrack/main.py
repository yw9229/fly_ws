import cv2
import numpy as np
from tracker.byte_tracker import BYTETracker

# 假的 YOLO 偵測器 (模擬)
def mock_yolo_detector(image):
    """
    模擬 YOLO 偵測輸出:
    假設這張影像有兩個偵測框
    格式: [x1, y1, x2, y2, score]
    """
    h, w, _ = image.shape
    dets = np.array([
        [0.3*w, 0.3*h, 0.6*w, 0.6*h, 0.9],  # 偵測框1
        [0.5*w, 0.5*h, 0.8*w, 0.8*h, 0.85], # 偵測框2
    ])
    return dets

# 設定 BYTETracker 參數 (可依需求調整)
class Args:
    track_thresh = 0.5
    track_buffer = 30
    match_thresh = 0.8
    mot20 = False

args = Args()
tracker = BYTETracker(args)

# 模擬一組圖片序列
images = [np.ones((480, 640, 3), dtype=np.uint8)*255 for _ in range(5)]  # 5張白底假圖
img_size = (640, 480)  # 偵測器 resize 輸入大小 (width, height)

for frame_id, image in enumerate(images):
    # 模擬 YOLO 偵測結果
    dets = mock_yolo_detector(image)

    # info_imgs = [height, width, scale]
    info_imgs = [image.shape[0], image.shape[1], 1.0]

    # 更新追蹤器
    online_targets = tracker.update(dets, info_imgs, img_size)

    # 取出追蹤結果
    print(f"Frame {frame_id+1}:")
    for track in online_targets:
        tlwh = track.tlwh        # [x, y, w, h]
        track_id = track.track_id
        score = track.score
        print(f"  ID: {track_id}, BBox: {tlwh}, Score: {score:.2f}")
