"""
Builds the single-page website (index.html) from web/template.html.

The page embeds three JSON blocks:
    data : a pre-computed default run, so the page shows something before the engine is ready
    py   : the Python sources of the stms package, run in the browser with Pyodide when no local server exists
    hc   : the hill-climbing result (experiment S6) shown in the algorithm panel
"""

import json
import os
import re
from typing import Any, Dict, Optional

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
TEMPLATE = os.path.join(ROOT, "web", "template.html")
SKIP_DIRS = {"visualization", "__pycache__"}          # matplotlib code is not needed in the browser
SKIP_FILES = {"site.py"}


def python_bundle() -> Dict[str, str]:
    files = {}
    pkg = os.path.join(ROOT, "stms")
    for d, dirs, names in os.walk(pkg):
        dirs[:] = sorted(x for x in dirs if x not in SKIP_DIRS)
        for n in sorted(names):
            if n.endswith(".py") and n not in SKIP_FILES:
                full = os.path.join(d, n)
                files[os.path.relpath(full, ROOT).replace(os.sep, "/")] = open(full, encoding="utf-8").read()
    return files


def hill_climbing_summary() -> Dict[str, Any]:
    from stms.demo import TUNED_PARAMS
    from stms.agents.intersection_agent import DEFAULT_PARAMS
    out = {"default": {k: DEFAULT_PARAMS[k] for k in TUNED_PARAMS}, "tuned": TUNED_PARAMS,
           "val_default": None, "val_tuned": None, "evaluations": None}
    path = os.path.join(ROOT, "results", "results.json")
    if os.path.exists(path):
        s6 = json.load(open(path, encoding="utf-8")).get("S6", {})
        out.update(val_default=s6.get("val_default_wait_s"), val_tuned=s6.get("val_tuned_wait_s"),
                   evaluations=s6.get("evaluations"))
    return out


def _embed(obj) -> str:
    return json.dumps(obj, separators=(",", ":")).replace("</", "<\\/")


def build_page(data: Optional[Dict[str, Any]] = None) -> str:
    from stms.demo import record
    html = open(TEMPLATE, encoding="utf-8").read()
    data = record() if data is None else data
    blocks = {"__DATA__": _embed(data), "__PY__": _embed(python_bundle()), "__HC__": _embed(hill_climbing_summary())}
    return re.sub(r"__(DATA|PY|HC)__", lambda m: blocks[m.group(0)], html)        # single pass


def write_site(path: Optional[str] = None) -> str:
    path = path or os.path.join(ROOT, "index.html")
    with open(path, "w", encoding="utf-8") as f:
        f.write(build_page())
    return path
