import os

LANG = "en"
exec(open(os.path.join(os.path.dirname(__file__), "..", "conf_common.py"), encoding="utf-8").read())
language = "en"
html_theme_options = theme(LANG)
