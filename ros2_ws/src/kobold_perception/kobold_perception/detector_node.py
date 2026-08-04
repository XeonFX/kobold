#!/usr/bin/env python3
"""YOLOv5s object detection on the RK3588 NPU.

WHY THIS IS THROTTLED
---------------------
Measured 2026-08-04 under realistic load (camera capturing, ASR transcribing
continuously, LLM generating):

    detector flat out (~40 fps)   ->  LLM 5.74 tok/s
    detector throttled to ~10 fps ->  LLM 7.25 tok/s   (+26%)

ASR was unaffected either way. The RKLLM artifact reports npu_core_num=3, so
it wants every core and RKNN_CORE_MASK does not constrain it -- detection and
generation contend directly on the NPU.

At 0.3 m/s the robot travels 3 cm between frames at 10 fps. Detecting a chair
leg 3 cm later is free; making every spoken reply 26% slower is not. If the
robot ever moves faster, raise `rate_hz` and expect to pay for it in tokens.

WHY RKNNLite AND NOT ctypes
---------------------------
Both work -- the version chain was first validated through ctypes precisely
because it has fewer moving parts. RKNNLite is used here for the ergonomics,
but note it needs /sys/firmware unmasked to identify the SoC (see
`security_opt: systempaths=unconfined` in compose.yaml). Docker applies bind
mounts BEFORE masking system paths, so mounting /proc/device-tree does not
work; unmasking does.
"""

from __future__ import annotations

import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

from . import rga

# YOLOv5s COCO. Kept inline rather than in a file: the class list is a property
# of the .rknn artifact, and a mismatch between them is a silent labelling bug.
COCO = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat",
    "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
    "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
    "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
    "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket",
    "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop",
    "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]

INPUT_SIZE = 640


def letterbox(image: np.ndarray, size: int = INPUT_SIZE):
    """Resize preserving aspect ratio, pad to square.

    Returns the padded image plus the scale and offsets needed to map boxes
    back to original image coordinates. Stretching instead of letterboxing
    would distort the aspect ratio the model was trained on and quietly cost
    accuracy, which is the sort of bug that never announces itself.
    """
    import cv2

    h, w = image.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = round(w * scale), round(h * scale)
    resized = cv2.resize(image, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)
    dx, dy = (size - nw) // 2, (size - nh) // 2
    canvas[dy : dy + nh, dx : dx + nw] = resized
    return canvas, scale, dx, dy


class DetectorNode(Node):
    def __init__(self) -> None:
        super().__init__("detector_node")

        self.declare_parameter("model_path", "/models/yolov5s_rk3588.rknn")
        self.declare_parameter("rate_hz", 10.0)
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("iou_threshold", 0.45)
        # 1 = NPU core 0. Vision gets one core; the LLM takes what it takes.
        self.declare_parameter("core_mask", 1)

        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.min_period = 1.0 / max(float(self.get_parameter("rate_hz").value), 0.1)

        self.bridge = CvBridge()
        self._last_run = 0.0
        self._latest: Image | None = None

        self._init_runtime()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(Image, "image_raw", self._on_image, qos)
        self.det_pub = self.create_publisher(Detection2DArray, "detections", 10)

        self.get_logger().info(
            f"detector ready: {self.get_parameter('model_path').value} "
            f"@ {self.get_parameter('rate_hz').value} Hz, core_mask="
            f"{self.get_parameter('core_mask').value}"
        )

    def _init_runtime(self) -> None:
        from rknnlite.api import RKNNLite

        model = self.get_parameter("model_path").value
        self.rknn = RKNNLite()
        if self.rknn.load_rknn(model) != 0:
            raise RuntimeError(f"load_rknn failed for {model}")

        core_mask = int(self.get_parameter("core_mask").value)
        if self.rknn.init_runtime(core_mask=core_mask) != 0:
            raise RuntimeError(
                "init_runtime failed. If this is inside a container, RKNNLite "
                "needs /sys/firmware unmasked to identify the SoC -- set "
                "security_opt: [systempaths=unconfined]. Bind-mounting "
                "/proc/device-tree does NOT work, because Docker applies bind "
                "mounts before masking system paths."
            )

    def _on_image(self, msg: Image) -> None:
        # Keep only the newest frame and run on a throttle. Queueing frames
        # would mean detecting where an obstacle *was*, which is the failure
        # this whole reactive layer exists to avoid.
        self._latest = msg
        now = time.monotonic()
        if now - self._last_run < self.min_period:
            return
        self._last_run = now
        self._run_once(msg)

    def _run_once(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # RGA when available: measured 0.69 ms cpu against cv2's 20.28 ms for
        # the same convert-and-letterbox, because cv2 gets its speed by using
        # every core. Falls back automatically.
        h, w = frame.shape[:2]
        padded = rga.bgr_letterbox(frame, INPUT_SIZE, INPUT_SIZE)
        scale, dx, dy = rga.letterbox_params(w, h, INPUT_SIZE, INPUT_SIZE)

        outputs = self.rknn.inference(inputs=[padded])
        boxes, scores, classes = self._postprocess(outputs)

        out = Detection2DArray()
        out.header = msg.header
        for (x1, y1, x2, y2), score, cls in zip(boxes, scores, classes):
            # Undo letterboxing back to source-image pixels.
            x1 = (x1 - dx) / scale
            x2 = (x2 - dx) / scale
            y1 = (y1 - dy) / scale
            y2 = (y2 - dy) / scale

            det = Detection2D()
            det.header = msg.header
            det.bbox = BoundingBox2D()
            det.bbox.center.position.x = float((x1 + x2) / 2.0)
            det.bbox.center.position.y = float((y1 + y2) / 2.0)
            det.bbox.size_x = float(abs(x2 - x1))
            det.bbox.size_y = float(abs(y2 - y1))

            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = COCO[cls] if cls < len(COCO) else str(cls)
            hyp.hypothesis.score = float(score)
            det.results.append(hyp)
            out.detections.append(det)

        self.det_pub.publish(out)

    def _postprocess(self, outputs):
        """Decode YOLOv5 heads and apply NMS.

        The three output tensors are the P3/P4/P5 heads at strides 8/16/32.
        Kept explicit rather than pulled from a helper library so the layout
        assumption is visible and checkable against the .rknn artifact.
        """
        anchors = [
            [(10, 13), (16, 30), (33, 23)],
            [(30, 61), (62, 45), (59, 119)],
            [(116, 90), (156, 198), (373, 326)],
        ]
        strides = [8, 16, 32]

        boxes, scores, classes = [], [], []
        for out, anchor_set, stride in zip(outputs, anchors, strides):
            arr = np.asarray(out)
            grid = INPUT_SIZE // stride
            arr = arr.reshape(3, -1, grid, grid).transpose(0, 2, 3, 1)

            obj = _sigmoid(arr[..., 4])
            keep = obj > self.conf_threshold
            if not np.any(keep):
                continue

            cls_scores = _sigmoid(arr[..., 5:])
            best_cls = np.argmax(cls_scores, axis=-1)
            best_score = np.max(cls_scores, axis=-1) * obj
            keep = best_score > self.conf_threshold
            if not np.any(keep):
                continue

            ai, gy, gx = np.nonzero(keep)
            xy = _sigmoid(arr[..., 0:2][keep]) * 2.0 - 0.5
            wh = (_sigmoid(arr[..., 2:4][keep]) * 2.0) ** 2

            cx = (xy[:, 0] + gx) * stride
            cy = (xy[:, 1] + gy) * stride
            aw = np.array([anchor_set[i][0] for i in ai])
            ah = np.array([anchor_set[i][1] for i in ai])
            bw, bh = wh[:, 0] * aw, wh[:, 1] * ah

            boxes.append(np.stack([cx - bw / 2, cy - bh / 2, cx + bw / 2, cy + bh / 2], axis=1))
            scores.append(best_score[keep])
            classes.append(best_cls[keep])

        if not boxes:
            return [], [], []

        boxes = np.concatenate(boxes)
        scores = np.concatenate(scores)
        classes = np.concatenate(classes)
        idx = _nms(boxes, scores, self.iou_threshold)
        return boxes[idx], scores[idx], classes[idx]

    def destroy_node(self) -> bool:
        if getattr(self, "rknn", None) is not None:
            self.rknn.release()
        return super().destroy_node()


def _sigmoid(x: np.ndarray) -> np.ndarray:
    return 1.0 / (1.0 + np.exp(-x))


def _nms(boxes: np.ndarray, scores: np.ndarray, threshold: float) -> list[int]:
    order = scores.argsort()[::-1]
    areas = (boxes[:, 2] - boxes[:, 0]) * (boxes[:, 3] - boxes[:, 1])
    keep: list[int] = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        xx1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        yy1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        xx2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        yy2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.maximum(0.0, xx2 - xx1) * np.maximum(0.0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-9)
        order = order[1:][iou <= threshold]
    return keep


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = DetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
