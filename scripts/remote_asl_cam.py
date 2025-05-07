import cv2
from dotenv import load_dotenv
from inference import get_model
import supervision as sv
from string import ascii_uppercase
import subprocess


from asl_cam.camera import Cam
from asl_cam.letter_builder import SentenceAssembler
from asl_cam.handler import generate_and_save_command

# ── Config & setup ──────────────────────────────────────────────────────
load_dotenv()  # loads ROBO_KEY and OPENAI_API_KEY from .env

API_KEY  = __import__("os").getenv("ROBO_KEY")
MODEL_ID = "american-sign-language-letters-gxpdm/4"

if not API_KEY:
    raise RuntimeError("YOOO no API key my guy")

model     = get_model(model_id=MODEL_ID, api_key=API_KEY)
cam       = Cam(index=0, width=640, height=640)
assembler = SentenceAssembler(gap_frames=30, stable_frames=15)
box_annot = sv.BoxAnnotator()
lab_annot = sv.LabelAnnotator()

print("Running remote ASL detector – press ESC to quit")

while True:
    frame = cam.read()

    # 1) Inference
    inf = model.infer(frame)[0]
    det = sv.Detections.from_inference(inf)

    # 2) Draw boxes & labels
    annotated = box_annot.annotate(frame.copy(), det)
    annotated = lab_annot.annotate(annotated, det)

    # 3) Extract top-1 letter
    class_ids = det.class_id
    letter    = ascii_uppercase[int(class_ids[0])] if class_ids.size > 0 else None
    display_letter = letter or ""

    # 4) Update assembler & handle completed words
    new_word = assembler.update(letter)
    if new_word:
        if new_word == "E":
            # Final “execute” trigger: build full command (drop the 'E')
            full_cmd = " ".join(assembler.words[:-1])
            print("Executing command:", full_cmd)
            # Generate and save the code file
            file_path = generate_and_save_command(full_cmd)
            print("Generated code saved to:", file_path)
            try:
                subprocess.run(["python3", "/Users/larsleopold/Documents/ASL_recognition_yolo/Real_time_asl_detector/bcast/cast.py"], check=True)
                print("cast.py executed successfully.")
            except subprocess.CalledProcessError as e:
                print(f"Error while running cast.py: {e}")
            assembler.reset()
        elif new_word == "Hello": #working progress- not quite working yet
            try:
                subprocess.run(["python3", "/Users/larsleopold/Documents/ASL_recognition_yolo/Real_time_asl_detector/bcast/stop_prosess.py"], check=True)
                print("stop_prosess executed successfully.")
            except subprocess.CalledProcessError as e: 
                print(f"Erik sin code suger: {e}")
            assembler.reset()
        else:
            print("Completed word:", new_word)

    # 5) Overlay sentence & current letter
    sentence = assembler.current_sentence()
    registred_letter = assembler.letter_history
    cv2.putText(annotated, sentence,       (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 1.7, (255,255,255), 3)
    cv2.putText(annotated, f"Registred letters: {registred_letter}", (10, 120),
                cv2.FONT_HERSHEY_SIMPLEX, 2.0, (0,255,0),     3)

    # 6) Show frame
    cv2.imshow("ASL Detector", annotated)
    if cv2.waitKey(1) & 0xFF == 27:
        break

# Cleanup
cam.release()
cv2.destroyAllWindows()