#!/usr/bin/env python
"""
semantic_threshold_test.py
──────────────────────────
Compute hit‑rate of the skill‑library's semantic lookup as the
similarity threshold sweeps from 0.75 → 0.95.

• Reads embeddings from skill_library/index.json
• Embeds each test phrase with the same OpenAI model
• Counts a HIT if max‑cosine ≥ threshold
• Prints a small table and writes threshold_hits.csv

No new skills are generated; the LLM is **not** invoked.
"""

import json, os, csv
import numpy as np
import openai
from pathlib import Path
import matplotlib.pyplot as plt

# ------------------------------------------------------------------
#  Paths and models (must match handler.py)
# ------------------------------------------------------------------
from asl_cam.handler import SKILL_DIR, INDEX_PATH
EMBED_MODEL = "text-embedding-3-small"
_oclient    = openai.OpenAI()

# ------------------------------------------------------------------
#  Paraphrase / query set
#   – you can freely extend or modify this list
# ------------------------------------------------------------------
PARAPHRASE_SET = [
    #   circle family
    "circle", "drive in a circle", "circle once", "spin around the spot",
    #   square family
    "square", "drive a square", "square drive",
    #   long combo
    "drive straight, then take a right, then a left, then a circle",
    #   misc
    "left", "line return"
]

NEGATIVE_SET = [
    # driving‑related but intentionally *not* in the library
    "zig‑zag path", "triangle drive", "figure eight",
    "spiral outwards", "random walk", "patrol the hallway",
    "small circle", "big circle", "double circle",
    "reverse circle", "drive a pentagon", "octagon route",
    "slalom between cones", "drive until wall then stop",
    "follow the line on the floor", "explore the room"
]

# ------------------------------------------------------------------
#  Load the skill index  (slug → {vector, …})
# ------------------------------------------------------------------
if not Path(INDEX_PATH).exists():
    raise RuntimeError(
        f"Index file not found at {INDEX_PATH}. Run the robot once to create skills first."
    )
index = json.loads(Path(INDEX_PATH).read_text())
skill_vectors = {
    slug: np.asarray(meta["vector"], dtype=np.float32)
    for slug, meta in index.items()
}

# ------------------------------------------------------------------
#  Helper: embed a sentence (returns np.ndarray[1536])
# ------------------------------------------------------------------
def embed(sentence: str) -> np.ndarray:
    resp = _oclient.embeddings.create(model=EMBED_MODEL, input=sentence)
    return np.asarray(resp.data[0].embedding, dtype=np.float32)

# ------------------------------------------------------------------
#  Similarity function (cosine)
# ------------------------------------------------------------------
def cosine(a: np.ndarray, b: np.ndarray) -> float:
    return float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b) + 1e-10)

# ------------------------------------------------------------------
#  Sweep thresholds and compute precision & recall
# ------------------------------------------------------------------
THRESHOLDS = [0.60, 0.65, 0.70, 0.75, 0.80, 0.85, 0.90, 0.95]
results    = []

for thresh in THRESHOLDS:
    TP = FP = FN = 0
    # positive (should match)
    for phrase in PARAPHRASE_SET:
        qvec      = embed(phrase)
        best_sim  = max(cosine(qvec, vec) for vec in skill_vectors.values())
        if best_sim >= thresh:
            TP += 1
        else:
            FN += 1
    # negative (should NOT match)
    for phrase in NEGATIVE_SET:
        qvec      = embed(phrase)
        best_sim  = max(cosine(qvec, vec) for vec in skill_vectors.values())
        if best_sim >= thresh:
            FP += 1
    precision = TP / (TP + FP + 1e-9)
    recall    = TP / (TP + FN + 1e-9)
    results.append((thresh, precision, recall, TP, FP, FN))

# ------------------------------------------------------------------
#  Print nicely and save CSV
# ------------------------------------------------------------------
print("\nPrecision & Recall vs. Threshold\n" + "-"*40)
print(f"{'Thresh':>6} | {'Precision':>9} | {'Recall':>6}")
for t,p,r,_,_,_ in results:
    print(f"{t:>6.2f} | {p:>9.2%} | {r:>6.2%}")
print()

with Path("threshold_hits.csv").open("w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["threshold", "precision", "recall", "TP", "FP", "FN"])
    for t,p,r,TP,FP,FN in results:
        writer.writerow([t, f"{p:.4f}", f"{r:.4f}", TP, FP, FN])

# ------------------------------------------------------------------
#  Plot Precision & Recall curves
# ------------------------------------------------------------------
thresholds = [t for t,_,_,_,_,_ in results]
precisions = [p for _,p,_,_,_,_ in results]
recalls    = [r for _,_,r,_,_,_ in results]

plt.figure()
plt.plot(thresholds, precisions, marker='o', label='Precision')
plt.plot(thresholds, recalls,    marker='o', label='Recall')
plt.xlabel('Cosine similarity threshold')
plt.ylabel('Score')
plt.ylim(0, 1.05)
plt.grid(True)
plt.legend()
plt.title('Precision–Recall vs. Threshold')
plt.tight_layout()
plt.savefig('precision_recall_curve.png', dpi=180)
print("Plot saved to precision_recall_curve.png")

print(f"CSV  written to threshold_hits.csv")