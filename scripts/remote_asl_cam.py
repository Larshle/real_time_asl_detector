from inference import get_model
import supervision as sv
from asl_cam.camera import Cam
import cv2, os

API_KEY   = os.getenv("ROBO_KEY")
MODEL_ID  = "american-sign-language-letters-gxpdm/4"

model = get_model(model_id=MODEL_ID, api_key=API_KEY)
cam   = Cam()
box_annot = sv.BoxAnnotator()
lab_annot = sv.LabelAnnotator()

print("Running get_model() – Esc to quit")
while True:
    frame = cam.read()

    # inference → list; take first item
    inf = model.infer(frame)[0]

    # convert to supervision Detections
    det = sv.Detections.from_inference(inf)

    # draw
    annotated = box_annot.annotate(frame.copy(), det)
    annotated = lab_annot.annotate(annotated, det)

    cv2.imshow("ASL get_model", annotated)
    if cv2.waitKey(1) & 0xFF == 27:
        break

cam.release(); cv2.destroyAllWindows()