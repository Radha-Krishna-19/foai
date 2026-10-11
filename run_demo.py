"""
Open the demo:     python run_demo.py
Builds demo.html (one self-contained page, works offline) and opens it in your browser.
    python run_demo.py --no-browser      just build the file
"""
import argparse
import json
import os
import sys
import webbrowser

ROOT = os.path.abspath(os.path.dirname(__file__))
sys.path.insert(0, ROOT)
from src.demo import record  # noqa: E402


def build(path=None):
    path = path or os.path.join(ROOT, "demo.html")
    page = open(os.path.join(ROOT, "src", "demo_page.html"), encoding="utf-8").read()
    with open(path, "w", encoding="utf-8") as f:
        f.write(page.replace("__DATA__", json.dumps(record())))
    return path


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-browser", action="store_true")
    a = ap.parse_args()
    p = build()
    print("Demo written to", p)
    if not a.no_browser:
        webbrowser.open("file://" + p.replace("\\", "/") if os.name == "nt" else "file://" + p)
