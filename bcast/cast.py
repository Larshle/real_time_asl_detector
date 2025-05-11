import subprocess
import os
from pathlib import Path
from dotenv import load_dotenv
load_dotenv()

# —— CONFIGURATION —— 
# Local generated command script:
local_path = Path.home() / "Documents/ASL_recognition_yolo/Real_time_asl_detector" \
             / "asl_cam/generated_commands/command.py"

# Remote SSH info:
USER       = os.getenv("USER_env")
HOST       = os.getenv("HOST_env")
KEY_PATH   = os.path.expanduser(os.getenv("KEY_PATH_env"))

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
    "cd /home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit && "
    "source devel/setup.bash && "
    "rosrun asl-pkg command.py"
)
remote_cmd1 = (
    "cd /home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit && source devel/setup.bash && rosrun goto-pkg goto_point.py _goal_x:=4 _goal_y:=2 _goal_yaw:=0"
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