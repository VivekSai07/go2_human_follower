"""YOLOv8 pose ONNX inference and person appearance extraction."""

from pathlib import Path

import cv2
import numpy as np

from .tracker import Candidate


def decode_pose_output(output: np.ndarray, image: np.ndarray, input_size: int = 640,
                       threshold: float = 0.5) -> list[Candidate]:
    data = np.asarray(output)
    if data.ndim == 3 and data.shape[0] == 1:
        data = data[0]
    if data.ndim != 2:
        raise ValueError("expected YOLOv8 pose tensor [56, N] or [N, 56]")
    if data.shape[0] == 56:
        data = data.T
    if data.shape[1] != 56:
        raise ValueError("expected COCO 17-keypoint YOLOv8 pose output")
    height, width = image.shape[:2]
    boxes, confidences, candidates = [], [], []
    for row in data:
        if row[4] < threshold:
            continue
        cx, cy, bw, bh = row[:4]
        x1 = float(max(0.0, (cx - bw / 2) * width / input_size))
        y1 = float(max(0.0, (cy - bh / 2) * height / input_size))
        x2 = float(min(width, (cx + bw / 2) * width / input_size))
        y2 = float(min(height, (cy + bh / 2) * height / input_size))
        if x2 <= x1 or y2 <= y1:
            continue
        keypoints = row[5:].reshape(17, 3)
        raised = any(keypoints[wrist, 2] > 0.5 and keypoints[shoulder, 2] > 0.5
                     and keypoints[wrist, 1] < keypoints[shoulder, 1] - 20
                     for wrist, shoulder in ((9, 5), (10, 6)))
        torso = image[int(y1 + (y2 - y1) * 0.15):int(y1 + (y2 - y1) * 0.7),
                      int(x1 + (x2 - x1) * 0.15):int(x2 - (x2 - x1) * 0.15)]
        if torso.size:
            hsv = cv2.cvtColor(torso, cv2.COLOR_BGR2HSV)
            hist = cv2.calcHist([hsv], [0, 1], None, [8, 8], [0, 180, 0, 256]).flatten()
            hist = hist / max(float(np.linalg.norm(hist)), 1e-6)
        else:
            hist = np.zeros(64)
        boxes.append([int(x1), int(y1), int(x2 - x1), int(y2 - y1)])
        confidences.append(float(row[4]))
        candidates.append(Candidate((x1, y1, x2, y2), hist, raised, float(row[4])))
    keep = cv2.dnn.NMSBoxes(boxes, confidences, threshold, 0.45)
    return [candidates[int(i)] for i in np.asarray(keep).flatten()] if len(keep) else []


class PoseDetector:
    def __init__(self, model_path: str, input_size: int = 640, threshold: float = 0.5):
        if not model_path or not Path(model_path).is_file():
            raise FileNotFoundError(f"pose ONNX model missing: {model_path!r}")
        self.net = cv2.dnn.readNetFromONNX(model_path)
        self.input_size = input_size
        self.threshold = threshold

    def detect(self, image: np.ndarray) -> list[Candidate]:
        blob = cv2.dnn.blobFromImage(image, scalefactor=1 / 255.0,
                                     size=(self.input_size, self.input_size), swapRB=True)
        self.net.setInput(blob)
        return decode_pose_output(self.net.forward(), image, self.input_size, self.threshold)
