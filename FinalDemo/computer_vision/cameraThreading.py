import cv2
from threading import Thread

class CameraThread(Thread):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_CONTRAST, 100)
        #self.cap.set(cv2.CAP_PROP_SATURATION, 100)
        #self.cap.set(cv2.CAP_PROP_EXPOSURE, -7)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        
        if not self.cap.isOpened():
            print("Error: Could not open camera.")
            #self.camera_avaliable = False
            return

        self.frame = None
        self.running = False
        
    def run(self):
        self.running = True
            
        while self.running:
            ret, frame = self.cap.read()
            if not ret:
                print("error: failed to read frame.")
                break
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            self.frame = frame
            

    def stop(self):
        self.running = False
        self.join()
        self.cap.release()
        print("camera released")
            


camera_thread = CameraThread()
camera_thread.start()
print(camera_thread)
while True:
    if camera_thread.frame is not None:
        frame = camera_thread.frame
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
cv2.destroyAllWindows()
