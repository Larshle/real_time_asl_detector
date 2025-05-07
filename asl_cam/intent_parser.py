import openai
from typing import Dict, Any
import os
from dotenv import load_dotenv
load_dotenv()
client = openai.OpenAI(api_key =  os.getenv("OPENAI_API_KEY"))

INTENT_PROMPT = """
You are a Python code generator for ROS1-code movement commands, important you code in ROS1 and not ROS2!. 
Given a instruction spelled in english, be aware there may be typos, 
produce only valid Python code (no explanations) that commands a robot to drive like the command. So say the command is circle make code for it to drive in a circle, if the command is drive in the shape of an E you drive the turtlebot like the shape of an E ect.
Keep in mind the turtlebot is in a small room, so dont make it drive to long, but always make it so the tutrlebot finishes driving the command. 
- The generated script should be complete and runnable on a TurtleBot.
Instruction: "{cmd}"
"""

def code_generator(cmd: str) -> Dict[str, Any]:
    resp = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[
            {"role":"system", "content":"You are a Python code generator for ROS 1 movement commands."},
            {"role":"user",   "content": INTENT_PROMPT.format(cmd=cmd)}
        ]
    )
    return resp.choices[0].message.content.strip()

def spell_check_word_with_gpt(spelled: str) -> str:
    prompt = (
        f"I captured these letters: {','.join(spelled)}. "
        "Return only the correctly spelled English word, nothing else."
    )
    resp = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[{"role":"user","content":prompt}]
    )
    return resp.choices[0].message.content.strip()