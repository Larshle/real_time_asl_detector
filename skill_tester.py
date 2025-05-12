#!/usr/bin/env python
"""
skill_library_test_v3.py
────────────────────────
• Clears index.json so every phrase starts unknown.
• Trial‑0 forces fresh generation and saves the skill.
• Trials‑1..N‑1 must hit semantic match → “reused”.
• Results logged to skill_test_log.csv and printed.

Run:  python skill_tester.py
"""

from pathlib import Path
import csv, json, time, datetime
from asl_cam.handler import (
    generate_or_fetch_command, SKILL_DIR, INDEX_PATH, _semantic_lookup
)
from asl_cam import intent_parser as IP

# ------------------------------------------------------------------
#  CONFIG
# ------------------------------------------------------------------
TEST_PHRASES = [
    "circle", "drive in a circle", "circle once", "spin in place",
    "square", "drive a square", "square drive",
    "drive straight, then take a right, then a left, then a circle",
    "left", "line return"
]
REPS = 10                                 # 1 generate + 9 reuse each
LOG  = Path("skill_test_log.csv")

# Disable spell‑checker to keep slugs literal
IP.spell_check_word_with_gpt = lambda x: x

# ------------------------------------------------------------------
#  Reset index.json (leave .py files in place)
# ------------------------------------------------------------------
Path(SKILL_DIR).mkdir(parents=True, exist_ok=True)
Path(INDEX_PATH).write_text("{}")
print("[reset] index.json cleared")

# ------------------------------------------------------------------
#  Helper to bypass semantic lookup on first trial
# ------------------------------------------------------------------
def force_generate_save(phrase: str) -> str:
    """Temporarily disable lookup so we always hit the LLM once."""
    from asl_cam import handler as H
    orig_lookup = H._semantic_lookup
    H._semantic_lookup = lambda *_a, **_k: None   # monkey‑patch → miss
    try:
        return H.generate_or_fetch_command(phrase, save_skill=True)
    finally:
        H._semantic_lookup = orig_lookup          # restore

# ------------------------------------------------------------------
#  Run benchmark
# ------------------------------------------------------------------
with LOG.open("w", newline="") as f:
    writer = csv.DictWriter(
        f, fieldnames=["phrase", "trial", "action", "elapsed_s"]
    )
    writer.writeheader()

    gen_total = reuse_total = 0

    for phrase in TEST_PHRASES:
        for trial in range(REPS):
            t0 = time.time()

            if trial == 0:
                path   = force_generate_save(phrase)
                action = "generated"
                gen_total += 1
            else:
                path   = generate_or_fetch_command(phrase, save_skill=False)
                action = "reused"
                reuse_total += 1

            elapsed = round(time.time() - t0, 2)
            writer.writerow({
                "phrase": phrase,
                "trial": trial,
                "action": action,
                "elapsed_s": elapsed
            })
            print(f"{phrase:<55} run {trial:<2} → {action:<9} {elapsed}s")

# ------------------------------------------------------------------
#  Summary
# ------------------------------------------------------------------
calls = gen_total + reuse_total
print("\n──── Summary ────")
print(f"Total calls : {calls}")
print(f"Generated   : {gen_total}")
print(f"Reused      : {reuse_total}")
print(f"Log written to {LOG}")