import cv2
import numpy as np
#from djitellopy import Tello

import time
import sys

from ultralytics import YOLO

CONFIDENCE = 0.5
font_scale = 1
thickness = 1
labels = open("/home/doganhasko/tello/object_detection_yolo/coco.names").read().strip().split("\n")
colors = np.random.randint(0, 255, size=(len(labels), 3), dtype="uint8")

model = YOLO("yolov8n.pt")
cap = cv2.VideoCapture(0)
#me = Tello()
#me.connect()
#me.streamoff()
#me.streamon()

while True:
    #image = me.get_frame_read().frame
    #image = cv2.resize(image, (416, 416))
    #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    _, image = cap.read()
    start = time.perf_counter()
    results = model.predict(image, conf=CONFIDENCE)[0]
    time_took = time.perf_counter() - start
    print("Time took:", time_took)

    for data in results.boxes.data.tolist():
        xmin, ymin, xmax, ymax, confidence, class_id = data
        xmin = int(xmin)
        ymin = int(ymin)
        xmax = int(xmax)
        ymax = int(ymax)
        class_id = int(class_id)

        color = [int(c) for c in colors[class_id]]
        cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color=color, thickness=thickness)
        text = f"{labels[class_id]}: {confidence:.2f}"
        (text_width, text_height) = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fontScale=font_scale, thickness=thickness)[0]
        text_offset_x = xmin
        text_offset_y = ymin - 5
        box_coords = ((text_offset_x, text_offset_y), (text_offset_x + text_width + 2, text_offset_y - text_height))
        overlay = image.copy()
        cv2.rectangle(overlay, box_coords[0], box_coords[1], color=color, thickness=cv2.FILLED)
        image = cv2.addWeighted(overlay, 0.6, image, 0.4, 0)
        cv2.putText(image, text, (xmin, ymin - 5), cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=font_scale, color=(0, 0, 0), thickness=thickness)

    end = time.perf_counter()
    fps = f"FPS: {1 / (end - start):.2f}"
    cv2.putText(image, fps, (50, 50),
                cv2.FONT_HERSHEY_SIMPLEX, 2, (0, 255, 0), 6)
    cv2.imshow("image", image)
    
    if ord("q") == cv2.waitKey(1):
        break

cv2.destroyAllWindows()
