import subprocess
from pathlib import Path
import os

# → path to your existing script
local_path = Path("/Users/larsleopold/Documents/ASL_recognition_yolo/Real_time_asl_detector/asl_cam/generated_commands/CIRCLE.py")

USER           = "ee106b-abk"
HOST           = "128.32.40.228"
REMOTE_DIR     = "/home/cc/ee106b/sp25/class/ee106b-abk/Documents/206B-projects/receive"
KEY_PATH       = "~/.ssh/robot6969"
RUN_AFTER_COPY = False

remote_path = f"{USER}@{HOST}:{REMOTE_DIR}/{local_path.name}"

# 1) Copy it:
subprocess.run([
    "scp", "-i", os.path.expanduser(KEY_PATH),
    str(local_path), remote_path
], check=True)

# 2) (Optionally) run it remotely:
if RUN_AFTER_COPY:
    subprocess.run([
        "ssh", "-i", os.path.expanduser(KEY_PATH),
        f"{USER}@{HOST}",
        f"python3 {REMOTE_DIR}/{local_path.name}"
    ], check=True)

print("Done.")