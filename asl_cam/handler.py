import os
from .intent_parser import parse_intent

def generate_and_save_command(code_text: str, output_dir: str = None) -> str:
    """
    Given a plain-text command, generate Python code via parse_intent,
    save it as a .py file in output_dir (default: ./generated_commands),
    and return the file path.
    """
    # Determine directory to save generated code
    if output_dir is None:
        output_dir = os.path.join(os.path.dirname(__file__), "generated_commands")
    os.makedirs(output_dir, exist_ok=True)

    # Generate code string
    code_str = parse_intent(code_text)

    # Create a sanitized filename
    key = code_text.strip().lower().replace(" ", "_")
    filename = f"{key}.py"
    file_path = os.path.join(output_dir, filename)

    # Write the code to the file
    with open(file_path, "w") as f:
        f.write(code_str)

    print(f"Generated command saved to: {file_path}")
    return file_path
