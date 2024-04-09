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
    
    results = model.predict(source=image, show=True, conf = conf)

    if len(results) > 1:
        raise Exception("More than one lego detected!")
    elif len(results) > 0:
        return [False, None]

    else: # only one lego found
        boxes = result[0].boxes.xywhn
        return [True, boxes]
            
            
    
    
