from __future__ import annotations
import json, os, shutil
from typing import Optional, Dict, Any

import numpy as np
import openai

from ... import intent_parser as IP   # re‑use OpenAI client & prompts

# ── Paths ──────────────────────────────────────────────────────────────
BASE_DIR   = os.path.dirname(__file__)
GEN_DIR    = os.path.join(BASE_DIR, "generated_commands")
SKILL_DIR  = os.path.join(BASE_DIR, "skill_library")
INDEX_PATH = os.path.join(SKILL_DIR, "index.json")

os.makedirs(GEN_DIR,   exist_ok=True)
os.makedirs(SKILL_DIR, exist_ok=True)

# ── Embedding helpers ──────────────────────────────────────────────────
EMBED_MODEL = "text-embedding-3-small"
_oclient    = openai.OpenAI()               # api_key picked up from env


def _embed(text: str) -> np.ndarray:
    """Return 1536‑D embedding vector (float32)."""
    resp = _oclient.embeddings.create(model=EMBED_MODEL, input=text)
    return np.asarray(resp.data[0].embedding, dtype=np.float32)


def _load_index() -> Dict[str, Any]:
    if os.path.exists(INDEX_PATH):
        with open(INDEX_PATH) as f:
            return json.load(f)
    return {}


def _save_index(idx: Dict[str, Any]) -> None:
    with open(INDEX_PATH, "w") as f:
        json.dump(idx, f, indent=2)


def _semantic_lookup(query: str, thresh: float = 0.85) -> Optional[str]:
    """Return skill path if cosine similarity ≥ thresh; else None."""
    idx = _load_index()
    if not idx:
        return None
    qvec = _embed(query)
    best_name, best_sim = None, 0.0
    for name, meta in idx.items():
        vec = np.asarray(meta["vector"], dtype=np.float32)
        sim = float(qvec @ vec) / (np.linalg.norm(qvec) * np.linalg.norm(vec))
        if sim > best_sim:
            best_name, best_sim = meta["path"], sim
    if best_sim >= thresh:
        print(f"[skill‑match] Reusing '{best_name}' (sim={best_sim:.2f})")
        return best_name
    return None


# ── Core function ──────────────────────────────────────────────────────
def generate_or_fetch_command(
    cmd_text: str,
    save_skill: bool | None = None,
) -> str:
    """
    Return a Python script path that implements `cmd_text`.

    Parameters
    ----------
    cmd_text   : raw sentence (already spelled‑checked upstream is OK)
    save_skill : True  → force saving as reusable skill
                 False → do not save
                 None  → no skill save (legacy default)

    Notes
    -----
    * The LLM is only called if no similar skill is found.
    * Markdown fences around the returned code are stripped automatically.
    """
    # 1) Try semantic reuse
    match = _semantic_lookup(cmd_text)      # may hit or miss
    if match:
        return match

    # 2) Generate new code via LLM
    cmd_text_spell = IP.spell_check_word_with_gpt(cmd_text)
    raw_code       = IP.code_generator(cmd_text_spell)

    # strip ``` fences / optional "python"
    lines = [ln for ln in raw_code.splitlines()
             if not ln.strip().startswith("```")
             and ln.strip().lower() != "python"]
    clean_code = "\n".join(lines)

    # save into generated_commands/ (overwrites each time)
    gen_path = os.path.join(GEN_DIR, "command.py")
    with open(gen_path, "w") as f:
        f.write(clean_code)
    print(f"[handler] New command written to {gen_path}")

    # 3) Persist as skill if requested
    if save_skill:
        slug      = "_".join(cmd_text_spell.lower().split())
        skill_py  = os.path.join(SKILL_DIR, f"{slug}.py")
        shutil.copy2(gen_path, skill_py)

        idx = _load_index()
        idx[slug] = {
            "path":   skill_py,
            "desc":   cmd_text_spell,
            "vector": _embed(cmd_text_spell).tolist()
        }
        _save_index(idx)
        print(f"[handler] Skill '{slug}' saved → {skill_py}")
        return skill_py

    return gen_path


# ── Legacy wrapper (keeps old imports working) ─────────────────────────
def generate_and_save_command(cmd_text: str, output_dir: str | None = None) -> str:
    """
    Legacy shim: behaves like the old version but WITHOUT automatic saving.
    The output_dir parameter is ignored; generation happens in generated_commands/.
    """
    if output_dir not in (None, GEN_DIR):
        print("[handler] Warning: output_dir param ignored in new handler.")
    return generate_or_fetch_command(cmd_text, save_skill=False)



#!/usr/bin/env python
"""
remote_asl_cam.py  –  ASL‑to‑TurtleBot pipeline *with* skill library support
────────────────────────────────────────────────────────────────────────────
• Detect ASL letters with Roboflow model
• Assemble words / sentences
• “E”  = execute command
• After execution, sign **Y** or **N** to decide if the script
  should be saved as a reusable skill.

Dependencies
────────────
opencv‑python, supervision, roboflow‑inference, python‑dotenv,
the rest of your existing repo (asl_cam.*)
"""

import cv2
import os
import subprocess
from string import ascii_uppercase

from dotenv import load_dotenv
from inference import get_model
import supervision as sv

from asl_cam.camera          import Cam
from asl_cam.letter_builder  import SentenceAssembler
from asl_cam.handler         import generate_or_fetch_command   # NEW

# ── Config & setup ──────────────────────────────────────────────────────
load_dotenv()  # loads ROBO_KEY and OPENAI_API_KEY

API_KEY  = os.getenv("ROBO_KEY")
MODEL_ID = "american-sign-language-letters-gxpdm-6id99/2"
if not API_KEY:
    raise RuntimeError("Missing ROBO_KEY in environment")

model     = get_model(model_id=MODEL_ID, api_key=API_KEY)
cam       = Cam(index=0, width=640, height=640)
assembler = SentenceAssembler(gap_frames=30, stable_frames=15)
box_annot = sv.BoxAnnotator()
lab_annot = sv.LabelAnnotator()

print("Running remote ASL detector – press ESC to quit")

# ── State vars for skill‑save handshake ────────────────────────────────
SAVE_PROMPT  = False          # waiting for Y / N ?
PENDING_CMD  = None           # str – full command sentence
PENDING_PATH = None           # str – tmp .py path (not yet saved)

# ── Main loop ───────────────────────────────────────────────────────────
while True:
    frame = cam.read()

    # 1) Inference
    inf = model.infer(frame)[0]
    det = sv.Detections.from_inference(inf)

    # 2) Draw boxes & labels
    annotated = lab_annot.annotate(
        box_annot.annotate(frame.copy(), det), det
    )

    # 3) Extract top‑1 letter
    class_ids      = det.class_id
    letter         = ascii_uppercase[int(class_ids[0])] if class_ids.size else None
    display_letter = letter or ""

    # 4) Sentence assembly
    new_word = assembler.update(letter)

    # ------------------------------------------------------------------
    # A) EXECUTE – saw “E” sentinel, build command
    # ------------------------------------------------------------------
    if not SAVE_PROMPT and new_word == "E":
        PENDING_CMD = " ".join(assembler.words[:-1])  # drop sentinel
        print(f"Command captured: '{PENDING_CMD}'")

        # Generate / reuse script WITHOUT saving yet
        PENDING_PATH = generate_or_fetch_command(PENDING_CMD, save_skill=False)
        SAVE_PROMPT  = True
        assembler.reset()
        print("Sign 'Y' to save as skill, 'N' to skip.")

    # ------------------------------------------------------------------
    # B) SAVE DECISION – expecting single Y or N
    # ------------------------------------------------------------------
    elif SAVE_PROMPT and new_word in {"Y", "N"}:
        save_flag = (new_word == "Y")
        final_path = generate_or_fetch_command(PENDING_CMD, save_skill=save_flag)

        # hand the chosen file to cast.py
        os.environ["ASL_CMD_FILE"] = final_path
        try:
            subprocess.run(
                ["python3",
                 "/Users/larsleopold/Documents/ASL_recognition_yolo/"
                 "Real_time_asl_detector/bcast/cast.py"],
                check=True
            )
            print("cast.py executed successfully.")
        except subprocess.CalledProcessError as e:
            print(f"Error while running cast.py: {e}")

        # reset state
        SAVE_PROMPT  = False
        PENDING_CMD  = None
        PENDING_PATH = None
        assembler.reset()

    # ------------------------------------------------------------------
    # C) Normal completed words (status only)
    # ------------------------------------------------------------------
    elif new_word:
        print("Completed word:", new_word)

    # 5) HUD overlay
    sentence_text = assembler.current_sentence()
    cv2.putText(annotated, sentence_text, (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255, 255, 255), 3)
    cv2.putText(annotated,
                f"Registered letters: {assembler.letter_history}",
                (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0, 255, 0), 3)

    # 6) Display
    cv2.imshow("ASL Detector", annotated)
    if cv2.waitKey(1) & 0xFF == 27:     # ESC
        break

# ── Cleanup ────────────────────────────────────────────────────────────
cam.release()
cv2.destroyAllWindows()
print("Shutdown complete.")
