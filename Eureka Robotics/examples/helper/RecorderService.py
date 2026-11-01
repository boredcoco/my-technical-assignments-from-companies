# recorder_service.py
import json, threading, time
from .Camera import Camera, CameraRecorder

class RecorderService:
    def __init__(self, camera_info, out_dir, scene, seed=None, events_path=None):
        self._camera_info = camera_info          # plain dict, shareable
        self._out_dir     = out_dir              # str
        self._scene       = scene
        self._seed        = seed
        self._events_path = events_path
        self._stop        = threading.Event()
        self._ready       = threading.Event()
        self._thread      = None
        self._ev_f        = None
        self._ev_lock     = threading.Lock()
        self.dropped      = 0

    def start(self):
        self._thread = threading.Thread(target=self._run, name="camera-recorder", daemon=True)
        self._thread.start()
        self._ready.wait(timeout=5)              # fail fast if open fails

    def _run(self):
        # Everything ZMQ + cv2 lives in THIS thread.
        self._cam = Camera(self._camera_info)    # socket born here
        self._rec = CameraRecorder(self._out_dir, self._camera_info,
                                   scene=self._scene, seed=self._seed)
        if self._events_path:
            self._ev_f = open(self._events_path, "a")
        self._ready.set()
        try:
            while not self._stop.is_set():
                parts = self._cam.recv_raw(timeout_ms=200)
                if parts is None:
                    continue
                gap = self._rec.write_raw(parts)
                if gap > 0:
                    self.dropped += gap
                    print(f"[camera] gap {gap}, total dropped {self.dropped}")
        finally:
            self._rec.close()
            self._cam.close()
            if self._ev_f:
                self._ev_f.close()

    def log_event(self, **fields):
        if self._ev_f is None:
            return
        fields["wall"] = time.time()
        with self._ev_lock:
            self._ev_f.write(json.dumps(fields) + "\n")
            self._ev_f.flush()

    def stop(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=5)
