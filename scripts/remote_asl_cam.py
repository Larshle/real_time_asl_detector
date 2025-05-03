import os
import cv2
from dotenv import load_dotenv
from inference import get_model
import supervision as sv
from string import ascii_uppercase

from asl_cam.camera import Cam
from asl_cam.letter_builder import SentenceAssembler

# ── Config & setup ──────────────────────────────────────────────────────
load_dotenv()  # loads ROBO_KEY from .env

API_KEY   = os.getenv("ROBO_KEY")
MODEL_ID  = "american-sign-language-letters-gxpdm/4"  

if not API_KEY:
    raise RuntimeError("Missing ROBO_KEY in environment")

# initialize model, camera, assembler, and annotators
model      = get_model(model_id=MODEL_ID, api_key=API_KEY)
cam        = Cam(index=0, width=640, height=640)
assembler = SentenceAssembler(gap_frames=35, stable_frames=15) # gap threshold for word break
box_annot  = sv.BoxAnnotator()
lab_annot  = sv.LabelAnnotator()

print("Running remote ASL detector – press ESC to quit")

# ── Main loop ───────────────────────────────────────────────────────────
while True:
    frame = cam.read()

    # 1) Inference
    inf = model.infer(frame)[0]            # first detection result
    det = sv.Detections.from_inference(inf)

    # 2) Draw boxes & labels
    annotated = box_annot.annotate(frame.copy(), det)
    annotated = lab_annot.annotate(annotated, det)

    # 3) Extract top-1 letter
    class_ids = det.class_id               # numpy array of ints
    if class_ids.size > 0:
        letter = ascii_uppercase[int(class_ids[0])]  # 0->'A', etc.
    else:
        letter = None

    # prepare display string for current letter
    display_letter = letter if letter is not None else ""

    # 4) Update word assembler & print completed words
    new_word = assembler.update(letter)
    if new_word:
        print("Completed word:", new_word)

    # 5) Overlay sentence & current letter
    sentence = assembler.current_sentence()
    # full sentence at top
    cv2.putText(
        annotated,
        sentence,
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (255, 255, 255),
        2,
    )
    # current letter below
    cv2.putText(
        annotated,
        f"Current: {display_letter}",
        (10, 70),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
    )

    # 6) Show the frame
    cv2.imshow("ASL Detector", annotated)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# ── Cleanup ───────────────────────────────────────────────────────────────
cam.release()
cv2.destroyAllWindows()