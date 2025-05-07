import subprocess, os, pathlib

USER       = "ee106b-acg"
HOST       = "128.32.40.228"
KEY_PATH   = os.path.expanduser("~/.ssh/robot6969")
REMOTE_WS  = "/home/cc/ee106b/sp25/class/ee106b-acg/Desktop/typeshit"
REMOTE_SCR = f"{REMOTE_WS}/src/asl-pkg/scripts"
LOCAL_CMD  = pathlib.Path("/…/generated_commands/command.py")
PID_FILE   = "remote_ros_pid.txt"

# 1) scp up the new command script
subprocess.run([
    "scp", "-i", KEY_PATH,
    str(LOCAL_CMD),
    f"{USER}@{HOST}:{REMOTE_SCR}/{LOCAL_CMD.name}"
], check=True)

# 2) ssh and launch in background under nohup, echo remote PID back
launch = (
    f"cd {REMOTE_WS} && "
    "source devel/setup.bash && "
    f"cd {REMOTE_SCR} && "
    "nohup rosrun asl-pkg command.py > /dev/null 2>&1 & "
    "echo $!"
)
# run and capture the remote PID
remote_pid = subprocess.check_output([
    "ssh", "-i", KEY_PATH, f"{USER}@{HOST}", launch
], text=True).strip()

# 3) save it locally
with open(PID_FILE, "w") as f:
    f.write(remote_pid)

print(f"Started remote ROS node, PID = {remote_pid}")