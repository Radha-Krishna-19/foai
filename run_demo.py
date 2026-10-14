"""
STMS demo - starts the website on a local Python server and opens it in the browser.

    python run_demo.py                 http://localhost:8000 (simulation runs in local Python)
    python run_demo.py --port 8080 --no-browser

The same page also works without this server (index.html on GitHub Pages or opened as a file):
it then runs the identical Python code inside the browser with Pyodide.
"""
import argparse
import os
import sys
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, ROOT)

from stms.demo import record_json  # noqa: E402
from stms.site import build_page  # noqa: E402


class Handler(BaseHTTPRequestHandler):
    page = None

    def log_message(self, *args):
        pass

    def _send(self, body: bytes, ctype: str, code: int = 200):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/api/ping":
            self._send(b"ok", "text/plain")
        elif u.path == "/api/run":
            q = {k: v[0] for k, v in parse_qs(u.query).items()}
            try:
                body = record_json(q.get("closed", ""), q.get("disp", ""), int(q.get("seed", "5")))
                self._send(body.encode(), "application/json")
            except Exception as e:                       # bad query -> readable error for the page
                self._send(str(e).encode(), "text/plain", 400)
        elif u.path in ("/", "/index.html"):
            if Handler.page is None:
                Handler.page = build_page().encode("utf-8")
            self._send(Handler.page, "text/html; charset=utf-8")
        else:
            self.send_error(404)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    srv = None
    for port in range(a.port, a.port + 20):          # an old demo window may still hold the port
        try:
            srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
            break
        except OSError:
            print(f"Port {port} is busy, trying {port + 1}")
    if srv is None:
        sys.exit("No free port found. Close old demo windows and try again.")
    url = f"http://localhost:{port}/"
    print(f"STMS demo running at {url}  (press Ctrl+C to stop)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
