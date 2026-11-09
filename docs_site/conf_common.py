"""Shared Sphinx configuration for the English (docs_site/en) and Chinese (docs_site/zh) docs.

Each language's conf.py sets LANG and execs this file. Build:
    sphinx-build -W -b html docs_site/en _site/docs          # English at /docs/
    sphinx-build -W -b html docs_site/zh _site/docs/zh       # Chinese at /docs/zh/
The published site (GitHub Pages) serves the LeRobotNeck showcase at / (projects/web_sim/web) and the full wallerobot
browser simulator at /sim/.
"""
import os

OWNER = "lauyihong"           # filled in by tools/export_public.py --owner
if "{" in OWNER:
    OWNER = "OWNER"
REPO = f"https://github.com/{OWNER}/wallerobot"
SITE = f"https://{OWNER}.github.io/wallerobot/"
DOCS = SITE + "docs/"

project = "wallerobot"
author = "wallerobot contributors"
copyright = "2026, wallerobot contributors (Apache-2.0)"
release = "0.1"

extensions = ["myst_parser", "sphinx_design", "sphinx_copybutton", "sphinx.ext.extlinks"]
# {repo}`path/in/repo` -> file on GitHub; {site}`sim/` -> the published site (title <target> works too)
extlinks = {"repo": (REPO + "/blob/main/%s", "%s"), "site": (SITE + "%s", None)}
myst_enable_extensions = ["colon_fence", "deflist", "attrs_inline", "dollarmath"]
myst_heading_anchors = 3
source_suffix = {".md": "markdown"}
exclude_patterns = ["_build"]

html_theme = "pydata_sphinx_theme"
html_title = "wallerobot"
html_static_path = ["../_static"]       # relative to the language folder
html_css_files = ["custom.css"]
html_show_sourcelink = False


def theme(lang):
    other = {"en": ("中文", "zh/"), "zh": ("English", "../")}[lang]
    sim_label = {"en": "Simulator", "zh": "模拟器"}[lang]
    return {
        "github_url": REPO,
        "navbar_align": "left",
        "show_toc_level": 2,
        "navigation_with_keys": False,
        "header_links_before_dropdown": 6,
        "external_links": [
            {"name": "LeRobotNeck", "url": SITE},
            {"name": sim_label, "url": SITE + "sim/"},
            {"name": other[0], "url": DOCS + ("zh/" if lang == "en" else "")},
        ],
        "footer_start": ["copyright"],
        "footer_end": [],
    }
