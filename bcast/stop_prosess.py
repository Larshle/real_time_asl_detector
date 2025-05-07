import subprocess, os, signal

USER     = "ee106b-acg"
HOST     = "128.32.40.228"
KEY_PATH = os.path.expanduser("~/.ssh/robot6969")
PID_FILE = "remote_ros_pid.txt"

if not os.path.exists(PID_FILE):
    raise RuntimeError(f"{PID_FILE} not found – nothing to stop.")

with open(PID_FILE) as f:
    pid = f.read().strip()

print(f"Killing remote process {pid}…")
# send SIGINT (or SIGTERM) to that PID on remote
subprocess.run([
    "ssh", "-i", KEY_PATH, f"{USER}@{HOST}",
    f"kill -{signal.SIGINT} {pid}"
], check=True)

os.remove(PID_FILE)
print("Done.")