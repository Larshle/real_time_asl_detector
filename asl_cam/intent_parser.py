import openai
from typing import Dict, Any
import os
from dotenv import load_dotenv
load_dotenv()
client = openai.OpenAI(api_key =  os.getenv("OPENAI_API_KEY"))

INTENT_PROMPT = """
You are a Python code generator for ROS1-code movement commands, important you code in ROS1 and not ROS2!. 
Given a instruction spelled in english, 
produce only valid Python code (no explanations) that commands a robot to drive like the command. So say the command is "circle" make code for it to drive in a circle, if the command is "drive in the shape of an E" you drive the turtlebot like the shape of an E ect.
- The generated script should be complete and runnable on a TurtleBot.
Make sure the turtlebot does not drive for ever, max for 20 sec 
"""

def code_generator(cmd: str) -> Dict[str, Any]:
    resp = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role":"system", "content":INTENT_PROMPT},
            {"role":"user",   "content": f"Instruction: {cmd}"}
        ]
    )
    return resp.choices[0].message.content.strip()

def spell_check_word_with_gpt(spelled: str) -> str:
    prompt = (
        f"I captured this sentence or word: {','.join(spelled)}. "
        "Return only the correctly spelled English word or sentence, nothing else."
    )
    resp = client.chat.completions.create(
        model="gpt-4o-mini",
        messages=[{"role":"user","content":prompt}]
    )
    return resp.choices[0].message.content.strip()

