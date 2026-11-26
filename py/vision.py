"""Pipe inspection / Raspberry Pi 4. Demo uses only the Python standard library."""
import argparse
import ctypes as C
import json
import math
import os
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
KINDS = ("clear", "crack", "deposit", "foreign")
CONF = 0.60


class Detection(C.Structure):
    _fields_ = [("frame", C.c_uint64), ("captured_ms", C.c_uint64),
                ("kind", C.c_int32), ("simulated", C.c_int32),
                ("confidence", C.c_float), ("bbox", C.c_float * 4),
                ("reserved", C.c_uint32)]


class Publisher:
    def __init__(self):
        libname = "pipeipc.dll" if os.name == "nt" else "libpipeipc.so"
        self.lib = C.CDLL(str(ROOT / "build" / libname))
        self.lib.ipc_clock_ms.restype = C.c_uint64
        self.lib.ipc_publish.argtypes = [C.POINTER(Detection)]
        self.lib.ipc_publish.restype = C.c_int
        if C.sizeof(Detection) != 48:
            raise RuntimeError("Unexpected IPC ABI")
        self.frame = 0

    def send(self, stamp, kind=0, confidence=0.0, bbox=(0, 0, 0, 0), simulated=True):
        self.frame += 1
        det = Detection(self.frame, stamp, kind, int(simulated), confidence,
                        (C.c_float * 4)(*bbox), 0)
        for _ in range(3):
            if self.lib.ipc_publish(C.byref(det)) == 0:
                return True
            time.sleep(0.002)
        return False


def simulation(pub, args):
    print("SIM AI:", args.scene, flush=True)
    while True:
        kind = KINDS.index(args.scene)
        if args.scene_file:
            try:
                kind = KINDS.index(args.scene_file.read_text(encoding="utf-8").strip())
            except (ValueError, OSError):
                time.sleep(0.1)
                continue
        stamp = pub.lib.ipc_clock_ms()
        pub.send(stamp, kind, 0.95 if kind else 0, simulated=True)
        time.sleep(1 / args.fps)


def check_names(names, mapping):
    ids = {int(i): mapping.get(str(label), str(label)) for i, label in names.items()}
    if not {"crack", "deposit", "foreign"} <= set(ids.values()):
        raise ValueError(f"Model needs crack/deposit/foreign labels or --labels mapping: {names}")
    return ids


def detect_camera(pub, args):
    import cv2
    from picamera2 import Picamera2
    from ultralytics import YOLO

    if not args.model or not args.model.exists():
        raise ValueError("--model must point to trained NCNN model directory")
    model = YOLO(str(args.model), task="detect")
    mapping = json.loads(args.labels.read_text(encoding="utf-8")) if args.labels else {}
    names = check_names(model.names, mapping)
    args.events.mkdir(parents=True, exist_ok=True)
    cam = Picamera2()
    cam.configure(cam.create_video_configuration(main={"size": (640, 480), "format": "RGB888"},
                                                buffer_count=3, queue=False))
    cam.start()
    last_saved = 0
    prev_kind = 0
    if args.preview:
        args.preview.parent.mkdir(parents=True, exist_ok=True)
    start = time.monotonic()
    try:
        while True:
            # Capture the first fresh frame from this request. Use call-start as
            # a conservative timestamp on the same monotonic clock used by C.
            stamp = pub.lib.ipc_clock_ms()
            frame = cam.capture_array()
            result = model.predict(frame, imgsz=args.imgsz, conf=CONF, verbose=False)[0]
            candidates = []
            for box in result.boxes:
                label = names[int(box.cls.item())]
                if label in KINDS[1:]:
                    score = float(box.conf.item())
                    coords = box.xyxyn[0].tolist()
                    candidates.append((label == "crack", score, KINDS.index(label), coords))
            if candidates:
                _, score, kind, coords = max(candidates, key=lambda c: (c[0], c[1]))
            else:
                score, kind, coords = 0.0, 0, (0, 0, 0, 0)
            pub.send(stamp, kind, score, coords, simulated=False)
            now = time.monotonic()
            save = kind and (kind != prev_kind or now - last_saved >= 2)
            if args.preview or save:
                annotated = result.plot()
            if args.preview:
                tmp = args.preview.with_suffix(".tmp.jpg")
                if cv2.imwrite(str(tmp), annotated):
                    os.replace(tmp, args.preview)
            if save:
                ts = datetime.now().astimezone()
                filename = f"{ts:%Y%m%d_%H%M%S_%f}_{KINDS[kind]}.jpg"
                if not cv2.imwrite(str(args.events / filename), annotated):
                    raise OSError("Cannot save event image")
                with (args.events / "events.jsonl").open("a", encoding="utf-8") as file:
                    print(json.dumps(dict(time=ts.isoformat(), frame=pub.frame, label=KINDS[kind],
                                          confidence=score, bbox=coords, image=filename)), file=file)
                last_saved = now
            prev_kind = kind
            if pub.frame % 10 == 0:
                print(f"mean FPS={pub.frame / (now - start):.2f}", flush=True)
    finally:
        cam.stop()
        cam.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=["sim", "camera"], default="sim")
    parser.add_argument("--scene", choices=KINDS, default="clear")
    parser.add_argument("--scene-file", type=Path)
    parser.add_argument("--fps", type=float, default=4)
    parser.add_argument("--model", type=Path)
    parser.add_argument("--labels", type=Path)
    parser.add_argument("--imgsz", type=int, default=320)
    parser.add_argument("--events", type=Path, default=Path("events"))
    parser.add_argument("--preview", type=Path, help="latest annotated .jpg for optional viewer")
    args = parser.parse_args()
    if not math.isfinite(args.fps) or not 0 < args.fps <= 60:
        parser.error("fps must be in (0,60]")
    # Hold a local-only singleton guard so two publishers cannot interleave frames.
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as guard:
        guard.bind(("127.0.0.1", 43772))
        pub = Publisher()
        if args.mode == "sim":
            simulation(pub, args)
        else:
            detect_camera(pub, args)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, RuntimeError, ValueError) as exc:
        sys.exit(str(exc))
