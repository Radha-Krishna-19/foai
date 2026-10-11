"""
Run the demo (interactive):     python run_demo.py
    Starts a small local page at http://localhost:8000 and opens it. Click any road to add or remove
    an accident; the real simulation re-runs each time.
    python run_demo.py --static     only write demo.html / index.html (default accident, no editing)
    python run_demo.py --port 8080 --no-browser
"""
import argparse
import json
import os
import pathlib
import sys
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, ROOT)
from src.demo import record, DEFAULT_CLOSED  # noqa: E402

PAGE = os.path.join(ROOT, "src", "demo_page.html")


def page_html(data) -> str:
    return open(PAGE, encoding="utf-8").read().replace("__DATA__", json.dumps(data))


def build(path=None):
    """Static copy with the default accident (used for GitHub Pages)."""
    path = path or os.path.join(ROOT, "demo.html")
    html = page_html(record())
    for p in {path, os.path.join(ROOT, "index.html")}:
        with open(p, "w", encoding="utf-8") as f:
            f.write(html)
    return path


def parse_closed(text):
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        if a.startswith("INT_") and b.startswith("INT_"):
            out.append((a, b))
    return out


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def _send(self, body: bytes, ctype: str):
        self.send_response(200)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        u = urlparse(self.path)
        if u.path == "/api/run":
            q = parse_qs(u.query)
            closed = DEFAULT_CLOSED if "default" in q else parse_closed(q.get("closed", [""])[0])
            self._send(json.dumps(record(closed)).encode(), "application/json")
        elif u.path in ("/", "/index.html"):
            self._send(page_html(record()).encode("utf-8"), "text/html; charset=utf-8")
        else:
            self.send_error(404)


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8000)
    ap.add_argument("--no-browser", action="store_true")
    ap.add_argument("--static", action="store_true")
    a = ap.parse_args()
    if a.static:
        print("Wrote", build())
        sys.exit()
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    url = f"http://localhost:{a.port}/"
    print(f"Demo running at {url}   (Ctrl+C to stop)")
    if not a.no_browser:
        webbrowser.open(url)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
