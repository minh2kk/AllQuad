# PlatformIO pre-build hook: regenerate src/web/web_ui_data.h from src/web/ui/ when a UI source is
# newer than the header (keeps the committed, fully minified header when nothing changed).
Import("env")  # noqa: F821
import glob
import os
import subprocess
import sys

root = env["PROJECT_DIR"]  # noqa: F821
header = os.path.join(root, "src", "web", "web_ui_data.h")
sources = glob.glob(os.path.join(root, "src", "web", "ui", "*")) + glob.glob(os.path.join(root, "tools", "i18n", "*")) + \
    [os.path.join(root, "tools", "embed_web.py")]
if not os.path.exists(header) or max(os.path.getmtime(p) for p in sources) > os.path.getmtime(header):
    subprocess.check_call([sys.executable, os.path.join(root, "tools", "embed_web.py")])
