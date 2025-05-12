from __future__ import annotations
import json, os, shutil
from typing import Optional, Dict, Any

import numpy as np
import openai

from . import intent_parser as IP   # re‑use OpenAI client & prompts

# ── Paths ──────────────────────────────────────────────────────────────
BASE_DIR   = os.path.dirname(__file__)
GEN_DIR    = os.path.join(BASE_DIR, "generated_commands")
SKILL_DIR  = os.path.join(BASE_DIR, "skill_library")
INDEX_PATH = os.path.join(SKILL_DIR, "index.json")

os.makedirs(GEN_DIR,   exist_ok=True)
os.makedirs(SKILL_DIR, exist_ok=True)

# ── Embedding helpers ──────────────────────────────────────────────────
EMBED_MODEL = "text-embedding-3-small"
_oclient    = openai.OpenAI()               


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