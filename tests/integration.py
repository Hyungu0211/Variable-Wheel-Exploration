"""Exercise the compiled controller via its real UDP and IPC boundaries."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "py"))
from station import Link
from vision import Publisher, Detection
from threading import Event, Thread
import ctypes as C

exe = ROOT / "build" / ("robot.exe" if os.name == "nt" else "robot")
port = 43779
proc = subprocess.Popen([str(exe), "--sim", "127.0.0.1", "127.0.0.1", str(port), "180", "450"])
link = Link("127.0.0.1", port)
pub = Publisher()
ai_stop = Event()
ai_kind = 0
auto_ai = True


def produce():
    while not ai_stop.is_set():
        if auto_ai:
            pub.send(pub.lib.ipc_clock_ms(), ai_kind, .95 if ai_kind else 0)
        ai_stop.wait(.08)


def expect(test, msg, timeout=2.6):
    last = None
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        last = link.send()
        if test(last):
            print("PASS", msg, flush=True)
            return
        time.sleep(.035)
    raise AssertionError((msg, last))


def rearm():
    expect(lambda s: s["reason"] == "DISARMED", "clear alone does not restart")
    time.sleep(.26)
    link.send("ARM")
    expect(lambda s: s["armed"], "explicit neutral rearm")


worker = Thread(target=produce, daemon=True)
try:
    time.sleep(.4)
    assert proc.poll() is None
    link.send("STOP")
    worker.start()
    rearm()
    link.send("DRIVE", 60)
    expect(lambda s: s["speed"] == 60, "drive")
    auto_ai = False
    time.sleep(.10)
    stamp=pub.lib.ipc_clock_ms()
    assert pub.send(stamp,3,.92)
    expect(lambda s:s["reason"]=="SLOW" and s["speed"]==30, "first distinct detection slows")
    for _ in range(5):
        time.sleep(.035)
        assert link.send()["armed"], "one frame counted twice"
    assert pub.send(pub.lib.ipc_clock_ms(),3,.92)
    expect(lambda s: s["reason"]=="BLOCKAGE" and s["speed"]==0, "second foreign frame stops")
    auto_ai=True
    rearm()
    link.send("DRIVE", -30)
    expect(lambda s:s["speed"]==-30,"reverse")
    ai_kind=1
    expect(lambda s:s["reason"]=="CRACK" and s["speed"]==0,"crack stop")
    ai_kind=0
    rearm()
    link.send("DRIVE",40)
    expect(lambda s:s["speed"]==40,"drive before wheel adjustment")
    link.send("WHEEL",250)
    expect(lambda s:s["reason"]=="WAIT_STOP" and s["wheel"]==0,"wheel waits for stationary feedback")
    expect(lambda s:s["wheel"]==250 and s["speed"]==0,"stopped wheel preset applied")
    link.send("DRIVE",40)
    expect(lambda s:s["reason"]=="ADJUSTING" and s["speed"]==0,"drive inhibited during wheel travel")
    expect(lambda s:s["speed"]==40,"drive after servo settling")
    link.send("SIMDIST",100)
    expect(lambda s:s["reason"]=="OBSTACLE" and not s["armed"] and s["speed"]==0,"ToF stop")
    link.send("SIMDIST",1000)
    rearm()
    link.send("DRIVE",30)
    expect(lambda s:s["speed"]==30,"drive before timeout")
    time.sleep(.72)
    state=link.send()
    assert not state["armed"] and state["speed"]==0 and state["reason"]=="CMD_TIMEOUT",state
    print("PASS command loss stops")
    rearm()
    auto_ai=False
    expect(lambda s:s["reason"]=="AI_STALE" and not s["armed"],"AI publisher loss stops")
    auto_ai=True
    rearm()
    # Verify the separate user-facing simulator can publish while our producer is stopped.
    auto_ai=False
    time.sleep(.12)
    demo=subprocess.Popen([sys.executable,str(ROOT/"py"/"vision.py"),"--scene","deposit","--fps","8"])
    try:
        expect(lambda s:s["reason"]=="BLOCKAGE" and not s["armed"],"standalone Python AI -> C IPC")
    finally:
        demo.terminate()
        demo.wait(timeout=5)
finally:
    ai_stop.set()
    if worker.is_alive(): worker.join(2)
    link.sock.close()
    if os.name=="nt": proc.terminate()
    else: proc.send_signal(signal.SIGTERM)
    proc.wait(timeout=5)
