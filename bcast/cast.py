import subprocess
from pathlib import Path
import os

local_path = Path("/Users/larsleopold/Documents/ASL_recognition_yolo/Real_time_asl_detector/asl_cam/generated_commands/command.py")

USER           = "ee106b-acg"
HOST           = "128.32.40.228"
REMOTE_DIR     = "/home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit/src/asl-pkg/scripts"
KEY_PATH       = "~/.ssh/robot6969"
RUN_AFTER_COPY = True

remote_cmd = (
    "cd /home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit && "
    "source devel/setup.bash && "
    "rosrun asl-pkg command.py"
)

remote_path = f"{USER}@{HOST}:{REMOTE_DIR}/{local_path.name}"
subprocess.run([
    "scp", "-i", os.path.expanduser(KEY_PATH),
    str(local_path), remote_path
], check=True)

if RUN_AFTER_COPY:
    subprocess.run(
        f"ssh -i {os.path.expanduser(KEY_PATH)} {USER}@{HOST} "
        f"\"bash -lc '{remote_cmd}'\"",
        shell=True,
        check=True
    )

print("Done.")