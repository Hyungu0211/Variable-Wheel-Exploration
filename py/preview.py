"""Serve the latest annotated JPEG from vision.py to an Ethernet-connected browser."""
import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("image",type=Path)
    p.add_argument("--bind",default="127.0.0.1")
    p.add_argument("--port",type=int,default=8080)
    args=p.parse_args()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):
            path=self.path.split("?")[0]
            if path=="/":
                data=b"""<!doctype html><meta charset=utf-8><title>Pipe camera</title>
                <h1>Pipe camera</h1><img id="frame" alt="Waiting for camera frame">
                <script>const f=document.getElementById('frame');
                function update(){f.src='/frame.jpg?t='+Date.now()}
                f.onload=f.onerror=()=>setTimeout(update,500);update()</script>"""
                mime="text/html; charset=utf-8"
            elif path=="/frame.jpg":
                try: data=args.image.read_bytes()
                except OSError:
                    self.send_error(503,"Waiting for camera");return
                mime="image/jpeg"
            else:
                self.send_error(404);return
            self.send_response(200)
            self.send_header("Content-Type",mime)
            self.send_header("Cache-Control","no-store")
            self.send_header("Content-Length",str(len(data)))
            self.end_headers()
            self.wfile.write(data)

    with ThreadingHTTPServer((args.bind,args.port),Handler) as server:
        server.serve_forever()
if __name__=="__main__":
    main()
