"""UDP ground station. Run on Pi or on the laptop connected by Ethernet."""
import argparse
import json
import socket
import threading


class Link:
    def __init__(self, host, port, timeout=0.25):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.connect((host, port))
        self.sock.settimeout(timeout)
        self.lock = threading.Lock()
        self.seq = 0

    def send(self, action="PING", value=0):
        with self.lock:
            self.seq += 1
            self.sock.send(f"PB1 {self.seq} {action} {value}".encode("ascii"))
            while True:
                state = json.loads(self.sock.recv(512))
                if state["seq"] == self.seq:
                    return state


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=43771)
    args = p.parse_args()
    link = Link(args.host, args.port)
    done = threading.Event()

    def heartbeat():
        while not done.wait(0.1):
            try:
                link.send()
            except (OSError, ValueError):
                pass

    th = threading.Thread(target=heartbeat, daemon=True)
    print("arm | drive -100..100 | stop | wheel 200/250 | status | simdist mm | quit")
    try:
        link.send("STOP")
        th.start()
        while True:
            parts = input("robot> ").strip().split()
            if not parts:
                continue
            op = parts[0].upper()
            if op == "QUIT":
                break
            try:
                if op in ("ARM", "STOP", "STATUS") and len(parts)==1:
                    print(link.send("PING" if op=="STATUS" else op))
                elif op in ("DRIVE", "WHEEL", "SIMDIST") and len(parts)==2:
                    print(link.send(op, int(parts[1])))
                else:
                    print("Unknown command")
            except (OSError, ValueError) as exc:
                print("Command reply:", exc)
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        done.set()
        if th.is_alive():
            th.join()
        try:
            link.send("STOP")
        except OSError:
            pass
        link.sock.close()


if __name__ == "__main__":
    main()
