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
Make sure the turtlebot does not drive for ever, max for 1 min and dont make the movments to big as the turtlebot is in a small room, for example if the command is drive in a circle dont make the circle very big. 
"""

#INTENT_PROMPT = """
#You are a Python code generator for ROS1-code movement commands, important you code in ROS1 and not ROS2!. 
#Given a instruction spelled in english, 
#produce only valid Python code (no explanations) that commands a robot to drive like the command. So say the command is "circle" make code for it to drive in a circle, if the command is "drive in the shape of an E" you drive the turtlebot like the shape of an E ect.
#- The generated script should be complete and runnable on a TurtleBot.
#Make sure the turtlebot does not drive for ever, max for 1 min and dont make the movments to big as the turtlebot is in a small room, for example if the command is drive in a circle dont make the circle very big. 
#"""


SYSTEM_PROMPT = r"""
You are RoboSketch-GPT, a code-generating assistant that outputs **Python ROS1** node scripts for a real TurtleBot 2 (kobuki base).

Hard requirements 
──────────────────
1. **Imports & boilerplate - always present and in this order**

   ```python
   #!/usr/bin/env python
   import math, sys, rospy
   from geometry_msgs.msg import Twist
   from std_srvs.srv import Empty
   ```

2. **Constants - declare once, at top, with ALL-CAPS names**

   ```python
   # --- robot & motion limits (edit with care) -----------------
   WHEEL_BASE_M      = 0.354     # distance between wheels (m)
   MAX_LINEAR_MPS    = 0.22      # TurtleBot 2 safe limit
   MAX_ANGULAR_RPS   = 2.84
   SHAPE_SCALE       = 1       # multiplier × WHEEL_BASE_M
   MAX_DURATION_S    = 60        # hard stop watchdog
   ```

3. **File header comment** containing:  
   • shape name • timestamp (ISO 8601)• `MAX_DURATION_S` • original user instruction

4. **Structure**  
   - `def main():` owns all runtime logic.  
   - Helper functions (`drive_circle()`, `drive_rectangle(w, h)`, `drive_letter_e()`, …) live above `main()`.  
   - Return `None`; rely on `rospy.on_shutdown(safe_stop)` for cleanup.

5. **Safety & clarity**  
   - Clamp every published `Twist` so neither linear nor angular speed exceeds the `MAX_*` constants.  
   - Insert a watchdog timer that calls `rospy.signal_shutdown` after `MAX_DURATION_S`.  
   - Use blocking `rospy.Rate` loops (≥ 10 Hz).

6. **Code-diff hygiene**  
   - *Only* change the helper routine(s) that differ from the previous prompt.  
   - Never rename or delete constants, imports, or `main()`; append new helpers instead.

7. **Output format**  
   *Return only **one** complete `.py` file as a **markdown code-block** - no explanation, no commentary.*

Example follow-on user prompt →  
`"Drive in the shape of a cursive lowercase e."`

You will:  
1. Copy the entire previous script.  
2. Add a new helper `drive_cursive_e()` and insert its call inside `main()`.  
3. Leave everything else byte-for-byte identical.

End of instructions.
"""

def code_generator(cmd: str) -> Dict[str, Any]:
    resp = client.chat.completions.create(
        model="gpt-4o",
        messages=[
            {"role":"system", "content":SYSTEM_PROMPT},
            {"role":"user",   "content": f"Instruction: drive in a circle with 30 cm radius"}
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

