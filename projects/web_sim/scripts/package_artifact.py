"""Turn the vite build of the full wallerobot simulator (web/dist/sim/index.html) into a claude.ai Artifact page.

The Artifact host wraps the published page in its own <!doctype><head><body> skeleton, so the page must be a fragment:
<title>, inline <style>, the body markup and the module <script>. Everything else in dist/ (JS bundle, wasm, model,
routes) is published next to it as supporting files at the same relative paths.

Writes web/dist/artifact_page.html and web/dist/artifact_files.json (the `files` map for the Artifact publish call).
Run after `npm run build`. The page has no data-root, so it loads model/ and routes/ next to itself (the site root).
The LeRobotNeck showcase (dist/index.html, play/, neck/, model_neck/, routes_neck/, img/) is not part of the Artifact.
"""
import base64
import gzip
import json
import re
from pathlib import Path

DIST = Path(__file__).resolve().parents[1] / "web" / "dist"
TYPES = {".wasm": "application/wasm", ".stl": "model/stl", ".xml": "application/xml", ".json": "application/json",
         ".js": "text/javascript", ".css": "text/css"}

html = (DIST / "sim" / "index.html").read_text(encoding="utf-8")
title = re.search(r"<title>.*?</title>", html, re.S).group(0)
js = re.search(r'<script type="module"[^>]*src="\.\./(assets/[^"]+\.js)"', html).group(1)
css = re.search(r'<link rel="stylesheet"[^>]*href="\.\./(assets/[^"]+\.css)"', html).group(1)
body = re.search(r"<body>(.*)</body>", html, re.S).group(1).strip()

page = "\n".join([
    title,
    "<style>",
    (DIST / css).read_text(encoding="utf-8"),
    "</style>",
    body,
    f'<script type="module" src="{js}"></script>',
    "",
])
(DIST / "artifact_page.html").write_text(page, encoding="utf-8")

# The Artifact host does not serve .stl: pack every mesh into one base64(gzip(concatenation)) text file and give the
# page a manifest that points at it (src/sim.js unpacks `packs` into MuJoCo's in-memory file system)
OUT = DIST / "artifact"
OUT.mkdir(exist_ok=True)
man = json.loads((DIST / "model" / "manifest.json").read_text(encoding="utf-8"))
meshes = [f for f in man["files"] if f.endswith(".stl")]
blob = b"".join((DIST / "model" / f).read_bytes() for f in meshes)
(OUT / "meshes.pack.txt").write_text(base64.b64encode(gzip.compress(blob, 9)).decode("ascii"), encoding="ascii")
man_a = {"entry": man["entry"], "files": [f for f in man["files"] if f not in meshes],
         "packs": [{"file": "meshes.pack.txt", "entries": [[f, (DIST / "model" / f).stat().st_size] for f in meshes]}]}
(OUT / "manifest.json").write_text(json.dumps(man_a), encoding="utf-8")

files = {"model/manifest.json": {"from": "artifact/manifest.json", "contentType": "application/json"},
         "model/meshes.pack.txt": {"from": "artifact/meshes.pack.txt", "contentType": "text/plain"}}
for p in sorted(DIST.rglob("*")):
    rel = p.relative_to(DIST).as_posix()
    if (p.is_dir() or rel.endswith("index.html") or rel in ("artifact_page.html", "artifact_files.json", "model/manifest.json")
            or rel.startswith(("model_neck/", "routes_neck/", "img/", "assets/app-")) or p.suffix == ".png"
            or rel == css or rel.startswith("artifact/") or p.suffix == ".stl"):
        continue
    files[rel] = {"from": rel, "contentType": TYPES[p.suffix.lower()]}
(DIST / "artifact_files.json").write_text(json.dumps(files, indent=1), encoding="utf-8")
size = sum((DIST / v["from"]).stat().st_size for v in files.values()) / 1e6
print(f"page {len(page) / 1e3:.1f} kB, {len(files)} supporting files, {size:.1f} MB")
