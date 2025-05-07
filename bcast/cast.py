import subprocess
import os
from pathlib import Path

# —— CONFIGURATION —— 
# Local generated command script:
local_path = Path.home() / "Documents/ASL_recognition_yolo/Real_time_asl_detector" \
             / "asl_cam/generated_commands/command.py"

# Remote SSH info:
USER       = "ee106b-acg"
HOST       = "128.32.40.228"
KEY_PATH   = os.path.expanduser("~/.ssh/robot6969")

# Remote workspace & script dir:
REMOTE_WS  = "/home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit"
REMOTE_SCR = REMOTE_WS + "/src/asl-pkg/scripts"

# Where to write the remote PID file:
REMOTE_PID = "/tmp/ros_command_pid"

# ── Build the remote launch command ──────────────────────────────────────
# 1) cd into workspace
# 2) source ROS setup
# 3) cd into scripts
# 4) run under nohup & background it
# 5) echo its PID into REMOTE_PID
remote_cmd = (
    f"cd {REMOTE_WS} && "
    "source devel/setup.bash && "
    f"cd {REMOTE_SCR} && "
    "nohup rosrun asl-pkg command.py > /dev/null 2>&1 & "
    # note the backslash before the $:
    f"echo \\$! > {REMOTE_PID}"
)

# ── 1) UPLOAD THE SCRIPT ───────────────────────────────────────────────────
print("Uploading command.py to remote host…")
subprocess.run([
    "scp", "-i", KEY_PATH,
    str(local_path),
    f"{USER}@{HOST}:{REMOTE_SCR}/{local_path.name}"
], check=True)
print("Upload complete.")

# ── 2) LAUNCH REMOTE NODE (DETACHED) ──────────────────────────────────────
print("Launching remote ROS node (detached)…")
ssh_call = (
    f"ssh -i {KEY_PATH} {USER}@{HOST} "
    f"\"bash -lc '{remote_cmd}'\""
)

# Use Popen so we don't block the local camera loop:
proc = subprocess.Popen(
    ssh_call,
    shell=True,
    stdout=subprocess.DEVNULL,
    stderr=subprocess.DEVNULL,
    preexec_fn=os.setsid
)

print("SSH command sent; returning immediately.")
print(f"Remote ROS node PID will be recorded in {REMOTE_PID} on the robot.")