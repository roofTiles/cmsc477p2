from ultralytics import YOLO
from robomaster import robot
from robomaster import camera

model = YOLO("models/OnlyGripperIsNull.pt")

# c is the desired class type for obj detection with the following
# options 'robot' and 'lego'

# conf is the desired confidence interval for detection
# image to be used for obj detection. If none given then
# it will use whatever is currently shown on the image.

# returns the bounding box array of format [x,y,w,h]
# normalized to original image size
def detect_object_in_image(c='robot', conf=0.8, image=None, ep_camera=None):
    if image == None:
        image = ep_camera.read_cv2_image(strategy="newest", timeout=5)

    classes = []

    if c == 'lego':
        classes=[0]
    if c == 'robot':
        classes = [2]
        
    results = model.predict(source=image, show=False, conf = conf,
                            imgsz=(384, 640), classes=classes)

    
    if len(results) > 1:
        raise Exception("More than one lego detected!")

    if len(results) == 1: # only one detection found
        result = results[0].cpu().numpy()
        boxes = result.boxes.xywhn

        if len(boxes) == 0: #nothing found
            return [False, None]

        bb = boxes[0]

        # scales to input pixel coords
        bb[0] = bb[0]*640
        bb[1] = bb[1]*384
        bb[2] = bb[2]*640
        bb[3] = bb[3]*384
        return [True, boxes[0]]
            
            
    
    
