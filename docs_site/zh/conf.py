import os

LANG = "zh"
exec(open(os.path.join(os.path.dirname(__file__), "..", "conf_common.py"), encoding="utf-8").read())
language = "zh_CN"
html_title = "wallerobot 文档"
html_theme_options = theme(LANG)
