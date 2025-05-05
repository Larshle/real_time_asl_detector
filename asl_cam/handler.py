import os
from . import intent_parser as IP

def generate_and_save_command(cmd_text: str, output_dir: str = None) -> str:
    """
    Given a plain-text command (e.g. "circle"), generate Python code via parse_intent,
    strip any Markdown fences and leading 'python' spec, save it as a .py file in output_dir
    (default: ./generated_commands), and return the file path.
    """
    # Determine directory to save generated code
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(__file__), "generated_commands")
    os.makedirs(output_dir, exist_ok=True)

    cmd_text_spell_checked = IP.spell_check_word_with_gpt(cmd_text)
    # Generate raw code string from the LLM
    code_str = IP.code_generator(cmd_text_spell_checked)

    # ── Cleanup: strip ``` fences and any 'python' spec ────────────────
    lines = code_str.splitlines()
    # remove opening fence (``` or ``` python)
    if lines and lines[0].strip().startswith("```"):
        lines = lines[1:]
    # remove closing fence
    if lines and lines[-1].strip().startswith("```"):
        lines = lines[:-1]
    # remove a standalone language spec line like 'python'
    if lines and lines[0].strip().lower() == "python":
        lines = lines[1:]
    code_str = "\n".join(lines)

    # Create a sanitized filename (e.g. "circle" → "circle.py")
    key      = cmd_text_spell_checked.strip().replace(" ", "_")
    filename = f"{key}.py"
    file_path = os.path.join(output_dir, filename)

    # Write the cleaned code to the file
    with open(file_path, "w") as f:
        f.write(code_str)

    print(f"[handler] Generated command saved to: {file_path}")
    return file_path