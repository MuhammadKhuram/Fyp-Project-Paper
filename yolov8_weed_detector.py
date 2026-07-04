"""
YOLOv8-Based Weed Detection and Classification
================================================
Paper: Autonomous Weed Removal using Delta Robot on Mobile Platform

Dataset: DeepWeeds + Custom UOG Agricultural Dataset
Classes: ['Broadleaf', 'Grassy', 'Sedge', 'Background']
Model: YOLOv8-nano (optimized for Jetson Orin NX edge deployment)
"""

import numpy as np
import time
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
from pathlib import Path


@dataclass
class WeedDetection:
    """Single weed detection result."""
    class_id: int
    class_name: str
    confidence: float
    bbox_xyxy: Tuple[float, float, float, float]   # pixel coords
    bbox_world: Optional[Tuple[float, float]] = None  # mm in robot frame
    weed_radius_mm: float = 0.0


WEED_CLASSES = {
    0: 'Broadleaf',
    1: 'Grassy',
    2: 'Sedge',
    3: 'Background'
}

# Removal priority (lower = higher priority)
REMOVAL_PRIORITY = {
    'Broadleaf': 1,
    'Grassy': 2,
    'Sedge': 1,
    'Background': 99,
}


class WeedDetector:
    """
    Weed detection pipeline using YOLOv8.

    Integrates:
    - YOLOv8-nano inference (Ultralytics)
    - Depth-based 3D localization (RealSense D435i)
    - Non-maximum suppression
    - Priority ranking for removal sequencing
    """

    def __init__(self,
                 model_path: str = 'models/yolov8n_weeds.pt',
                 conf_threshold: float = 0.45,
                 iou_threshold: float = 0.45,
                 img_size: int = 640,
                 device: str = 'cuda'):
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.img_size = img_size
        self.device = device
        self.model_path = Path(model_path)
        self._model = None
        self._inference_times = []

        # Camera intrinsics (RealSense D435i @ 640x480)
        self.fx = 618.0  # focal length x (pixels)
        self.fy = 618.0  # focal length y
        self.cx = 320.0  # principal point x
        self.cy = 240.0  # principal point y

    def load_model(self):
        """Load YOLOv8 model (lazy load)."""
        try:
            from ultralytics import YOLO
            self._model = YOLO(str(self.model_path))
            self._model.to(self.device)
            print(f"[WeedDetector] Model loaded: {self.model_path}")
        except ImportError:
            print("[WeedDetector] ultralytics not installed — running in simulation mode")
        except FileNotFoundError:
            print(f"[WeedDetector] Model not found at {self.model_path} — simulation mode")

    def detect(self, rgb_image: np.ndarray,
               depth_image: Optional[np.ndarray] = None) -> List[WeedDetection]:
        """
        Run weed detection on an RGB image.

        Args:
            rgb_image: HxWx3 uint8 array
            depth_image: HxW float32 depth in meters (from RealSense)

        Returns:
            List of WeedDetection objects, sorted by removal priority
        """
        t0 = time.perf_counter()

        if self._model is None:
            # Simulation mode: generate synthetic detections
            detections = self._simulate_detections(rgb_image)
        else:
            detections = self._run_inference(rgb_image)

        # 3D localization if depth available
        if depth_image is not None:
            detections = self._localize_3d(detections, depth_image)

        # Sort by priority then confidence
        detections.sort(key=lambda d: (REMOVAL_PRIORITY.get(d.class_name, 99), -d.confidence))

        elapsed = (time.perf_counter() - t0) * 1000
        self._inference_times.append(elapsed)
        return detections

    def _run_inference(self, image: np.ndarray) -> List[WeedDetection]:
        """Run actual YOLOv8 inference."""
        results = self._model.predict(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.img_size,
            verbose=False
        )
        detections = []
        for r in results:
            for box in r.boxes:
                cid = int(box.cls.item())
                det = WeedDetection(
                    class_id=cid,
                    class_name=WEED_CLASSES.get(cid, 'Unknown'),
                    confidence=float(box.conf.item()),
                    bbox_xyxy=tuple(box.xyxy[0].tolist())
                )
                detections.append(det)
        return detections

    def _simulate_detections(self, image: np.ndarray) -> List[WeedDetection]:
        """Generate synthetic detections for testing without a GPU model."""
        np.random.seed(42)
        n_det = np.random.randint(3, 12)
        h, w = image.shape[:2]
        detections = []
        for _ in range(n_det):
            cid = np.random.choice([0, 1, 2], p=[0.45, 0.35, 0.20])
            conf = np.random.uniform(0.55, 0.98)
            x1 = np.random.uniform(0, w - 60)
            y1 = np.random.uniform(0, h - 60)
            x2 = x1 + np.random.uniform(30, 80)
            y2 = y1 + np.random.uniform(30, 80)
            detections.append(WeedDetection(
                class_id=cid,
                class_name=WEED_CLASSES[cid],
                confidence=conf,
                bbox_xyxy=(x1, y1, min(x2, w), min(y2, h))
            ))
        return detections

    def _localize_3d(self, detections: List[WeedDetection],
                      depth: np.ndarray) -> List[WeedDetection]:
        """Convert pixel bounding boxes to 3D world coordinates (mm)."""
        for det in detections:
            x1, y1, x2, y2 = det.bbox_xyxy
            cx_px = (x1 + x2) / 2.0
            cy_px = (y1 + y2) / 2.0
            # Sample depth at center of bounding box (5x5 median)
            r0, r1 = max(0, int(cy_px)-2), min(depth.shape[0], int(cy_px)+3)
            c0, c1 = max(0, int(cx_px)-2), min(depth.shape[1], int(cx_px)+3)
            patch = depth[r0:r1, c0:c1]
            if patch.size == 0 or np.all(patch == 0):
                continue
            z_m = float(np.median(patch[patch > 0]))  # meters
            # Back-project to 3D (camera frame, mm)
            x_mm = (cx_px - self.cx) * z_m / self.fx * 1000.0
            y_mm = (cy_px - self.cy) * z_m / self.fy * 1000.0
            det.bbox_world = (x_mm, y_mm)
            # Estimate weed radius from bounding box width
            px_width = x2 - x1
            det.weed_radius_mm = (px_width / self.fx) * z_m * 500.0
        return detections

    @property
    def avg_inference_time_ms(self) -> float:
        """Average inference time over all processed frames."""
        return float(np.mean(self._inference_times)) if self._inference_times else 0.0

    def benchmark(self, n_frames: int = 100) -> dict:
        """Run a benchmark to measure inference speed."""
        dummy_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        self._inference_times.clear()
        for _ in range(n_frames):
            self.detect(dummy_img)
        times = np.array(self._inference_times)
        return {
            'n_frames': n_frames,
            'mean_ms': float(np.mean(times)),
            'std_ms': float(np.std(times)),
            'min_ms': float(np.min(times)),
            'max_ms': float(np.max(times)),
            'fps': 1000.0 / float(np.mean(times))
        }


# ─── Self-test ────────────────────────────────────────────────────────────────
if __name__ == '__main__':
    print("YOLOv8 Weed Detector — Simulation Test")
    print("=" * 45)
    detector = WeedDetector()
    dummy = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    dummy_depth = np.random.uniform(0.3, 1.5, (480, 640)).astype(np.float32)
    detections = detector.detect(dummy, dummy_depth)
    print(f"Detected {len(detections)} weeds:")
    for i, d in enumerate(detections):
        loc = f"({d.bbox_world[0]:.0f}, {d.bbox_world[1]:.0f})mm" if d.bbox_world else "unknown"
        print(f"  [{i+1}] {d.class_name:12s} conf={d.confidence:.3f}  loc={loc}")
    bm = detector.benchmark(50)
    print(f"\nBenchmark (n=50): mean={bm['mean_ms']:.1f}ms ± {bm['std_ms']:.1f}ms  fps={bm['fps']:.1f}")
