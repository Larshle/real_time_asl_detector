import openai
from typing import Dict, Any
import os
from dotenv import load_dotenv
load_dotenv()
client = openai.OpenAI(api_key =  os.getenv("OPENAI_API_KEY"))

INTENT_PROMPT = """
You are a Python code generator for ROS2-style movement commands. 
Given a single instruction spelled in english  (e.g., "square", "circle", "triangle") be aware there may be typos, 
produce only valid Python code (no explanations) that commands a robot to drive that shape or like the command.
You choose how big the shape the turtlebot drives in, but keep them not to big so the bot kan drive in a small room.
- Use rclpy and geometry_msgs/Twist to publish to 'cmd_vel'.
- The generated script should be complete and runnable on a TurtleBot.
Instruction: "{cmd}"
"""

def parse_intent(cmd: str) -> Dict[str, Any]:
    resp = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role":"system", "content":"You emit a single JSON intent."},
            {"role":"user",   "content": INTENT_PROMPT.format(cmd=cmd)}
        ]
    )
    import json
    return json.loads(resp.choices[0].message.content)