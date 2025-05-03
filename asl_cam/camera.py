import cv2, platform

class Cam:
    def __init__(self, index=0, width=640, height=640):
        backend = cv2.CAP_ANY
        if platform.system() == "Darwin":
            backend = cv2.CAP_AVFOUNDATION
        elif platform.system() == "Linux":
            backend = cv2.CAP_V4L2  # or CAP_GSTREAMER on Jetson
        self.cap = cv2.VideoCapture(index, backend)
        if not self.cap.isOpened():
            raise RuntimeError("Camera init failed.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

    def read(self):
        ok, frame = self.cap.read()
        if not ok:
            raise RuntimeError("No frame.")
        return frame

    def release(self):
        self.cap.release()