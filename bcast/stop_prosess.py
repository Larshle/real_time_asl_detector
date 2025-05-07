#!/usr/bin/env python3
import subprocess
import os
import sys
import signal

# —— CONFIGURATION (must match cast.py) —— 
USER       = "ee106b-acg"
HOST       = "128.32.40.228"
KEY_PATH   = os.path.expanduser("~/.ssh/robot6969")
REMOTE_PID = "/tmp/ros_command_pid"

# 1) Read the remote PID
ssh_cat = (
    f"ssh -i {KEY_PATH} {USER}@{HOST} "
    f"\"cat {REMOTE_PID} 2>/dev/null || echo ''\""
)
pid = subprocess.check_output(ssh_cat, shell=True, text=True).strip()
if not pid:
    print("No remote PID file found; is the node running?")
    sys.exit(1)
print(f"Found remote ROS PID: {pid}")

# 2) Try SIGTERM
print("Sending SIGTERM...")
ssh_term = [
    "ssh", "-i", KEY_PATH, f"{USER}@{HOST}",
    f"kill -SIGTERM {pid}"
]
subprocess.run(ssh_term, check=True)

# 3) Check if it's still alive, then SIGKILL
ssh_check = (
    f"ssh -i {KEY_PATH} {USER}@{HOST} "
    f"\"kill -0 {pid} 2>/dev/null && kill -SIGKILL {pid} || echo 'Process exited cleanly'\""
)
subprocess.run(ssh_check, shell=True, check=True)
print("SIGKILL sent if it was still running.")

# 4) Remove the PID file
ssh_rm = [
    "ssh", "-i", KEY_PATH, f"{USER}@{HOST}",
    f"rm {REMOTE_PID}"
]
subprocess.run(ssh_rm, check=True)
print("Removed remote PID file. Done.")