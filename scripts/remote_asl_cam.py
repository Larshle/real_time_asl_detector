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
MODEL_ID = "american-sign-language-letters-gxpdm/4"
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
        print("Saved to skills")
        # hand the chosen file to cast.py
        os.environ["ASL_CMD_FILE"] = final_path
        #try:
        #    subprocess.run(
        #        ["python3",
        #         "/Users/larsleopold/Documents/ASL_recognition_yolo/"
         #        "Real_time_asl_detector/bcast/cast.py"],
         #       check=True
          #  )
         #   print("cast.py executed successfully.")
        #except subprocess.CalledProcessError as e:
          #  print(f"Error while running cast.py: {e}")

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
