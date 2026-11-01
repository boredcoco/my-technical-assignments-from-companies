"""
Camera.py — ZMQ RGBD camera client + recorder for the vacuum simulator.

Classes:
    Camera          : subscribes to the camera stream, decodes frames,
                      provides pixel->world / point-cloud helpers.
    CameraRecorder  : writes rgb.mp4 (or rgb.avi) + depth.u16 + frames.jsonl
                      + session.json for offline use.

Functions:
    unwrap_rpc(x)         : normalise an RPC reply to a plain dict
    read_recording(dir)   : generator over a saved recording

Conventions
    - RGB images are BGR (OpenCV). Swap channels before Qt/RGB consumers.
    - Depth values are metres (float32); 0 = invalid/out-of-range.
    - World frame per the simulator spec; yaw CCW from +x.

Threading
    A ZMQ socket must be created, used and destroyed on one thread.
    Construct `Camera` inside the thread that will read from it.
"""

import json
import math
import os
import time

import cv2
import numpy as np
import zmq


# --------------------------------------------------------------------------
# RPC reply unwrapping
# --------------------------------------------------------------------------
def unwrap_rpc(x):
    """
    Accept whatever an RPC helper returns and produce the inner result dict.

    Handles:
        (header_dict, *payload_frames)    recv_multipart reply
        (header_dict,)                    single-frame reply
        b'...' / '...'                    raw JSON
        {"ok": True,  "result": {...}}    envelope
        {"ok": False, "error":  "..."}    -> raises RuntimeError
        already-unwrapped dict
    """
    if isinstance(x, tuple):
        if not x:
            raise TypeError("empty RPC reply")
        x = x[0]

    if isinstance(x, (bytes, bytearray)):
        x = x.decode("utf-8")
    if isinstance(x, str):
        x = json.loads(x)

    if isinstance(x, dict):
        if x.get("ok") is False:
            raise RuntimeError(f"RPC failed: {x.get('error')}")
        if "ok" in x and "result" in x:
            x = x["result"]

    if not isinstance(x, dict):
        raise TypeError(
            f"expected dict after unwrap, got {type(x).__name__}: {x!r}")
    return x


# --------------------------------------------------------------------------
# Streaming client
# --------------------------------------------------------------------------
class Camera:
    """RGBD camera subscriber with geometry helpers."""

    def __init__(self, camera_info, endpoint="tcp://localhost:5558",
                 hwm=200, context=None):
        camera_info = unwrap_rpc(camera_info)
        self.info = camera_info

        self.W = int(camera_info["width"])
        self.H = int(camera_info["height"])
        self.fx = float(camera_info["fx"])
        self.fy = float(camera_info["fy"])
        self.cx = float(camera_info["cx"])
        self.cy = float(camera_info["cy"])
        self.depth_scale = float(camera_info["depth_scale"])
        self.T_base_camera = np.asarray(camera_info["T_base_camera"],
                                        dtype=np.float64)

        self._ctx = context if context is not None else zmq.Context.instance()
        self._sub = self._ctx.socket(zmq.SUB)
        self._sub.setsockopt(zmq.SUBSCRIBE, b"")
        self._sub.setsockopt(zmq.RCVHWM, int(hwm))
        self._sub.connect(endpoint)

    @classmethod
    def from_rpc(cls, rpc_call, endpoint="tcp://localhost:5558", **kw):
        """rpc_call(method, **params) -> anything unwrap_rpc can handle."""
        return cls(rpc_call("get_camera_info"), endpoint=endpoint, **kw)

    def set_conflate(self):
        """Keep only the newest frame. Use for visualisation, never recording."""
        self._sub.setsockopt(zmq.CONFLATE, 1)
        self._sub.setsockopt(zmq.RCVHWM, 1)

    def recv_raw(self, timeout_ms=1000):
        """Return (header_bytes, jpg_bytes, depth_u16_bytes) or None on timeout."""
        if not self._sub.poll(timeout_ms):
            return None
        return self._sub.recv_multipart()

    def decode(self, parts):
        """(hdr_json_bytes, jpg, raw_u16) -> (hdr, bgr HxWx3, depth_m HxW float32)."""
        hdr_raw, jpg, raw = parts
        hdr = json.loads(hdr_raw)
        bgr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        depth = (np.frombuffer(raw, dtype="<u2")
                   .reshape(hdr["height"], hdr["width"])
                   .astype(np.float32) * self.depth_scale)
        return hdr, bgr, depth

    def latest(self, timeout_ms=1000):
        """Decoded (hdr, bgr, depth_m) or None on timeout."""
        parts = self.recv_raw(timeout_ms)
        return None if parts is None else self.decode(parts)

    def pixel_to_world(self, hdr, depth_m, u, v):
        """Single pixel -> world (x, y, z), or None if depth invalid."""
        z = float(depth_m[v, u])
        if z <= 0.0:
            return None
        x_c = (u - self.cx) * z / self.fx
        y_c = (v - self.cy) * z / self.fy
        p_robot = self.T_base_camera @ np.array([x_c, y_c, z, 1.0])
        pose = hdr["pose"]
        c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        return (pose["x"] + c * p_robot[0] - s * p_robot[1],
                pose["y"] + s * p_robot[0] + c * p_robot[1],
                float(p_robot[2]))

    def depth_to_pointcloud(self, hdr, depth_m):
        """World-frame (N, 3) float32 point cloud; invalid pixels dropped."""
        H, W = depth_m.shape
        vs, us = np.mgrid[0:H, 0:W]
        valid = depth_m > 0
        if not valid.any():
            return np.zeros((0, 3), dtype=np.float32)
        z = depth_m[valid]
        x = (us[valid] - self.cx) * z / self.fx
        y = (vs[valid] - self.cy) * z / self.fy
        pts_c = np.stack([x, y, z, np.ones_like(z)], axis=1)
        pts_r = pts_c @ self.T_base_camera.T
        pose = hdr["pose"]
        c, s = math.cos(pose["yaw"]), math.sin(pose["yaw"])
        xw = pose["x"] + c * pts_r[:, 0] - s * pts_r[:, 1]
        yw = pose["y"] + s * pts_r[:, 0] + c * pts_r[:, 1]
        return np.stack([xw, yw, pts_r[:, 2]], axis=1).astype(np.float32)

    def close(self):
        try:
            self._sub.close(0)
        except Exception:
            pass


# --------------------------------------------------------------------------
# Video writer with codec fallback
# --------------------------------------------------------------------------
# Order matters: prefer MP4/H.264 if the build supports it, fall back to
# MJPG/.avi which is built into every OpenCV.
_CODEC_CANDIDATES = [
    ("mp4v", ".mp4"),
    ("avc1", ".mp4"),
    ("H264", ".mp4"),
    ("MJPG", ".avi"),
    ("XVID", ".avi"),
]


def _open_video_writer(out_dir, fps, width, height):
    """
    Try each codec/extension pair until VideoWriter opens.
    Returns (writer, filename, codec). Raises RuntimeError if none work.
    """
    tried = []
    for codec, ext in _CODEC_CANDIDATES:
        filename = os.path.join(out_dir, f"rgb{ext}")
        fourcc = cv2.VideoWriter_fourcc(*codec)
        writer = cv2.VideoWriter(filename, fourcc, float(fps), (width, height))
        if writer.isOpened():
            return writer, filename, codec
        writer.release()
        tried.append(f"{codec}{ext}")

    build = cv2.getBuildInformation()
    ffmpeg_line = next(
        (ln for ln in build.splitlines() if "FFMPEG" in ln), "FFMPEG: (not found)"
    )
    raise RuntimeError(
        "could not open VideoWriter for any codec. Tried: "
        + ", ".join(tried)
        + f".\n{ffmpeg_line.strip()}\n"
        "Fix: install a full OpenCV build (e.g. `pip install opencv-python`) "
        "or use MJPG/.avi (built into OpenCV)."
    )


# --------------------------------------------------------------------------
# Recorder
# --------------------------------------------------------------------------
class CameraRecorder:
    """
    Write a self-describing recording:
        session.json   one-off metadata (camera intrinsics, scene, seed, codec)
        rgb.mp4 / .avi decoded RGB frames at nominal fps
        depth.u16      concatenated raw depth frames (LE uint16, mm)
        frames.jsonl   one line per frame: seq, t, wall, pose, offsets, gap
    """

    def __init__(self, out_dir, camera_info, scene="?", seed=None, fps=15):
        camera_info = unwrap_rpc(camera_info)
        out_dir = os.path.abspath(out_dir)          # macOS VideoWriter prefers abs paths
        self.out_dir = out_dir
        self.info = camera_info
        self.W = int(camera_info["width"])
        self.H = int(camera_info["height"])
        self.fps = float(fps)

        os.makedirs(out_dir, exist_ok=True)
        if not os.access(out_dir, os.W_OK):
            raise RuntimeError(f"output directory is not writable: {out_dir}")

        # 1. Open the video writer first. If this fails, we raise before
        #    claiming a recording exists.
        self._vw, self._rgb_path, self._rgb_codec = _open_video_writer(
            out_dir, self.fps, self.W, self.H
        )

        # 2. Write session.json with the *actual* codec and filename.
        meta = {
            "scene": scene,
            "seed": seed,
            "t0_wall": time.time(),
            "camera_info": camera_info,
            "rgb_fps": self.fps,
            "rgb_file": os.path.basename(self._rgb_path),
            "rgb_codec": self._rgb_codec,
            "depth": {
                "file": "depth.u16",
                "dtype": "<u2",
                "width": self.W,
                "height": self.H,
                "scale": camera_info["depth_scale"],
                "invalid": 0,
            },
            "index": "frames.jsonl",
        }
        with open(os.path.join(out_dir, "session.json"), "w") as f:
            json.dump(meta, f, indent=2)

        # 3. Open the depth and index streams.
        self._depth_f = open(os.path.join(out_dir, "depth.u16"), "wb")
        self._index_f = open(os.path.join(out_dir, "frames.jsonl"), "w")
        self._last_seq = None
        self._frame_i = 0
        self._closed = False

    def write_raw(self, parts):
        """
        Feed one multipart message. Returns the number of frames missed
        before this one (>= 0), or -1 if the RGB frame could not be decoded.
        """
        if self._closed:
            raise RuntimeError("CameraRecorder already closed")
        hdr_raw, jpg, raw_depth = parts
        hdr = json.loads(hdr_raw)

        offset = self._depth_f.tell()
        self._depth_f.write(raw_depth)

        bgr = cv2.imdecode(np.frombuffer(jpg, np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            self._vw.write(bgr)

        gap = 0 if self._last_seq is None else hdr["seq"] - self._last_seq - 1
        rec = {
            "seq": hdr["seq"],
            "t": hdr["t"],
            "wall": time.time(),
            "pose": hdr["pose"],
            "rgb_frame": self._frame_i if bgr is not None else None,
            "depth_offset": offset,
            "depth_bytes": len(raw_depth),
            "gap": gap,
        }
        self._index_f.write(json.dumps(rec) + "\n")
        self._last_seq = hdr["seq"]
        if bgr is not None:
            self._frame_i += 1
        return -1 if bgr is None else gap

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._vw.release()
        self._depth_f.close()
        self._index_f.close()


# --------------------------------------------------------------------------
# Playback
# --------------------------------------------------------------------------
def read_recording(out_dir):
    """Yield one dict per frame: {seq, t, pose, gap, rgb (BGR), depth (m)}."""
    out_dir = os.path.abspath(out_dir)
    with open(os.path.join(out_dir, "session.json")) as f:
        meta = json.load(f)

    W = int(meta["depth"]["width"])
    H = int(meta["depth"]["height"])
    scale = float(meta["depth"]["scale"])
    rgb_file = os.path.join(out_dir, meta.get("rgb_file", "rgb.mp4"))

    cap = cv2.VideoCapture(rgb_file)
    if not cap.isOpened():
        raise RuntimeError(f"could not open video: {rgb_file}")

    depth_f = open(os.path.join(out_dir, meta["depth"]["file"]), "rb")
    try:
        with open(os.path.join(out_dir, meta.get("index", "frames.jsonl"))) as f:
            for line in f:
                rec = json.loads(line)
                bgr = None
                if rec.get("rgb_frame") is not None:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, rec["rgb_frame"])
                    ok, frame = cap.read()
                    bgr = frame if ok else None
                depth_f.seek(rec["depth_offset"])
                raw = depth_f.read(rec["depth_bytes"])
                depth = (np.frombuffer(raw, "<u2")
                           .reshape(H, W).astype(np.float32) * scale)
                yield {**rec, "rgb": bgr, "depth": depth}
    finally:
        cap.release()
        depth_f.close()
