from asl_cam.camera import Cam
import cv2, time

cam = Cam()
frames, t0 = 0, time.time()
while True:
    frame = cam.read()
    cv2.imshow("preview", frame)
    frames += 1
    if time.time() - t0 >= 1:
        print("FPS:", frames); frames=0; t0=time.time()
    key = cv2.waitKey(1) & 0xFF
    if key == 27:
        break
cam.release(); cv2.destroyAllWindows()