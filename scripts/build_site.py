"""Writes index.html (the website, also used by GitHub Pages).   Usage: python scripts/build_site.py"""
import os
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
sys.path.insert(0, ROOT)

from stms.site import write_site  # noqa: E402

if __name__ == "__main__":
    print("Wrote", write_site())
