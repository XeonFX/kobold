#!/usr/bin/env python3
"""IMX219 capture for Kobold, publishing sensor_msgs/Image.

WHY GSTREAMER AND NOT cv2.VideoCapture(0)
-----------------------------------------
Two independent reasons, both discovered the hard way:

  * /dev/video11 (the rkisp mainpath) is a *multiplanar* V4L2 device, and
  * the OpenCV in the perception image is built WITHOUT the V4L2 backend.

So the GStreamer path is not a preference, it is the only one that works. It
also needs gstreamer1.0-plugins-good present in the image or you get
`no element "v4l2src"` at runtime -- see Dockerfile.perception.

WHY 4:3 AND NOT 1080p
---------------------
The IMX219 array is 3280x2464 (4:3) at 1.12 um pitch behind a 3.04 mm lens,
giving 62.2 deg horizontal. There are three ways to get a smaller frame and
they are NOT equivalent:

  downscale   read the whole array, let the ISP scale   -> keeps 62.2 deg
  bin         combine 2x2 pixels, whole array           -> keeps 62.2 deg
  crop        read a centre window                      -> 1920 wide = 38.9 deg

Asking for 1920x1080 from a 4:3 sensor also crops vertically, taking ~48.8 deg
down to ~37 -- and that crop comes off the top and bottom, which is exactly
where the floor immediately in front of the robot lives. A robot needs to see
the ground it is about to drive onto more than it needs a cinematic aspect
ratio, so capture 4:3 and accept the framerate.

WHY EXPOSURE IS DRIVEN EXPLICITLY
---------------------------------
Measured 2026-08-04: rkaiq's 3A converges once when a stream starts and then
holds. Two completely different scenes in one session both sat at
exposure=2100 / analogue_gain=1536, and brightness was flat across all 300
frames of a capture. Driving from a lit room into a shadow therefore keeps the
exposure chosen at startup. `auto_exposure: false` takes the controls back.
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image

from . import rga

# Sensor geometry, from the IMX219 datasheet and the Radxa Camera 8M 219
# product brief. Used to synthesise a plausible CameraInfo until a real
# calibration exists.
SENSOR_W_PX = 3280
SENSOR_H_PX = 2464
PIXEL_UM = 1.12
FOCAL_MM = 3.04


def build_pipeline(device: str, width: int, height: int, raw_nv12: bool) -> str:
    """GStreamer pipeline string for the ISP mainpath.

    drop=true with max-buffers=1 makes the sink always hand us the newest
    frame. For a robot a stale frame is worse than a dropped one -- acting on
    where an obstacle *was* is the failure we are avoiding.

    With RGA available we take NV12 straight off the ISP and convert on the
    2D accelerator instead of asking GStreamer's videoconvert to do it. That
    was measured at 37.9% of a core versus 5.4% for the raw path -- roughly
    32% of an A55 handed back, on the cluster the camera shares with the whole
    ROS graph.
    """
    tail = ("" if raw_nv12 else "videoconvert ! video/x-raw,format=BGR ! ")
    return (
        f"v4l2src device={device} io-mode=4 ! "
        f"video/x-raw,format=NV12,width={width},height={height} ! "
        f"{tail}"
        f"appsink drop=true max-buffers=1 sync=false"
    )


class CameraNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_node")

        self.declare_parameter("device", "/dev/video11")
        self.declare_parameter("subdev", "/dev/v4l-subdev2")
        self.declare_parameter("width", 1280)
        self.declare_parameter("height", 960)
        self.declare_parameter("frame_id", "camera_optical_frame")
        # Publish rate cap. The sensor delivers what it delivers; this only
        # stops us republishing the same frame faster than it arrives.
        self.declare_parameter("max_rate_hz", 15.0)
        # rkaiq does not track the scene (see module docstring). False means we
        # own exposure and gain.
        self.declare_parameter("auto_exposure", False)
        self.declare_parameter("exposure", 2100)
        self.declare_parameter("analogue_gain", 1536)
        # Orientation is a property of how the module is bolted to the chassis,
        # so it belongs in config, not code. Flipping an IMX219 also changes
        # the Bayer order -- if colours go strange after setting these, rotate
        # downstream instead.
        self.declare_parameter("hflip", False)
        self.declare_parameter("vflip", False)
        # Frames to discard after the stream opens, while 3A settles.
        self.declare_parameter("warmup_frames", 20)
        # Falls back to CPU conversion automatically if the RGA is unavailable.
        self.declare_parameter("use_rga", True)

        self.device = self.get_parameter("device").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.bridge = CvBridge()

        # Sensor-data QoS: best effort, keep last. A control loop that blocks
        # waiting for a retransmitted camera frame is a robot that stops.
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.image_pub = self.create_publisher(Image, "image_raw", qos)
        self.info_pub = self.create_publisher(CameraInfo, "camera_info", qos)

        self._apply_sensor_controls()

        # Only ask GStreamer for BGR if we cannot do it on the RGA.
        self.use_rga = bool(self.get_parameter("use_rga").value) and rga.available()
        self.get_logger().info(rga.status())
        pipeline = build_pipeline(self.device, self.width, self.height,
                                  raw_nv12=self.use_rga)
        self.get_logger().info(f"opening: {pipeline}")
        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            raise RuntimeError(
                f"could not open {self.device} via GStreamer. Check that "
                "gstreamer1.0-plugins-good is installed (provides v4l2src) and "
                "that the container has the device and the video group."
            )

        warmup = int(self.get_parameter("warmup_frames").value)
        for _ in range(warmup):
            self.cap.read()
        self.get_logger().info(f"camera up: {self.width}x{self.height}, discarded {warmup} warmup frames")

        self.info = self._build_camera_info()
        period = 1.0 / max(float(self.get_parameter("max_rate_hz").value), 1.0)
        self.timer = self.create_timer(period, self._tick)
        self._fail_streak = 0

    # -- sensor controls ----------------------------------------------------
    def _apply_sensor_controls(self) -> None:
        """Set exposure/gain/orientation directly on the sensor subdev.

        Uses v4l2-ctl rather than a Python binding because the subdev controls
        are not exposed through the capture device, and shelling out once at
        startup is cheaper than carrying another dependency.
        """
        import shutil
        import subprocess

        subdev = self.get_parameter("subdev").value
        if not shutil.which("v4l2-ctl"):
            self.get_logger().warning("v4l2-ctl not found; leaving sensor controls at defaults")
            return

        ctrls = [
            f"horizontal_flip={int(bool(self.get_parameter('hflip').value))}",
            f"vertical_flip={int(bool(self.get_parameter('vflip').value))}",
        ]
        if not self.get_parameter("auto_exposure").value:
            ctrls += [
                f"exposure={int(self.get_parameter('exposure').value)}",
                f"analogue_gain={int(self.get_parameter('analogue_gain').value)}",
            ]

        cmd = ["v4l2-ctl", "-d", subdev, "--set-ctrl", ",".join(ctrls)]
        result = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            # Not fatal: a wrong subdev path or a kernel that renumbered the
            # nodes should degrade to "camera works, exposure is whatever 3A
            # picked", not "no camera".
            self.get_logger().warning(
                f"could not set sensor controls ({' '.join(cmd)}): {result.stderr.strip()}"
            )
        else:
            self.get_logger().info(f"sensor controls: {', '.join(ctrls)}")

    # -- camera info --------------------------------------------------------
    def _build_camera_info(self) -> CameraInfo:
        """Synthesise CameraInfo from sensor geometry.

        This is NOT a calibration. It is a geometrically consistent guess so
        that depth projection and TF have something to work with before a
        checkerboard calibration exists. Distortion is left zero; the Radxa
        module specifies TV-distortion <0.3%, which is small enough to ignore
        until it demonstrably is not.
        """
        # Effective focal length in pixels, accounting for ISP scaling from the
        # full array to our output width.
        focal_px_full = FOCAL_MM / (PIXEL_UM / 1000.0)
        scale = self.width / float(SENSOR_W_PX)
        fx = fy = focal_px_full * scale
        cx, cy = self.width / 2.0, self.height / 2.0

        info = CameraInfo()
        info.header.frame_id = self.frame_id
        info.width = self.width
        info.height = self.height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        return info

    # -- main loop ----------------------------------------------------------
    def _tick(self) -> None:
        ok, frame = self.cap.read()
        if ok and frame is not None and self.use_rga:
            # appsink hands NV12 back as (H*3/2, W) single channel.
            frame = rga.nv12_to_bgr(frame.reshape(self.height * 3 // 2, self.width),
                                    self.width, self.height)
        if not ok or frame is None:
            self._fail_streak += 1
            if self._fail_streak in (1, 10, 100):
                self.get_logger().warning(f"capture read failed ({self._fail_streak} in a row)")
            return
        self._fail_streak = 0

        stamp = self.get_clock().now().to_msg()
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        self.info.header.stamp = stamp
        self.info_pub.publish(self.info)

    def destroy_node(self) -> bool:
        if getattr(self, "cap", None) is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
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
