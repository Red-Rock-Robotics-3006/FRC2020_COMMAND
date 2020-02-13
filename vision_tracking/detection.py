import numpy as np
from time import time
import json
import sys
from edgetpu.detection.engine import DetectionEngine
from PIL import Image
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer
from networktables import NetworkTablesInstance
import cv2


def parseError(str, config_file):
    """Report parse error."""
    print("config error in '" + config_file + "': " + str, file=sys.stderr)


def read_config(config_file):
    """Read configuration file."""
    team = -1

    # parse file
    try:
        with open(config_file, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(config_file, err), file=sys.stderr)
        return team

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object", config_file)
        return team

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number", config_file)

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras", config_file)

    return team


class PBTXTParser:
    def __init__(self, path):
        self.path = path
        self.file = None

    def parse(self):
        with open(self.path, 'r') as f:
            self.file = ''.join([i.replace('item', '') for i in f.readlines()])
            blocks = []
            obj = ""
            for i in self.file:
                if i == '}':
                    obj += i
                    blocks.append(obj)
                    obj = ""
                else:
                    obj += i
            self.file = blocks
            label_map = {}
            for obj in self.file:
                obj = [i for i in obj.split('\n') if i]
                i = int(obj[1].split()[1]) - 1
                name = obj[2].split()[1][1:-1]
                label_map.update({i: name})
            self.file = label_map

    def get_labels(self):
        return self.file


def log_object(obj, labels):
    print('-----------------------------------------')
    if labels:
        print(labels[obj.label_id])
    print("score = {:.3f}".format(obj.score))
    box = obj.bounding_box.flatten().tolist()
    print("box = [{:.3f}, {:.3f}, {:.3f}, {:.3f}]".format(*box))


def main(config):
    
    team = read_config(config)
    WIDTH, HEIGHT = 320, 240

    print("Connecting to Network Tables")
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(team)

    """Format of these entries found in WPILib documentation."""
    nb_objects_entry = ntinst.getTable("ML").getEntry("nb_objects")
    boxes_entry = ntinst.getTable("ML").getEntry("boxes")
    object_classes_entry = ntinst.getTable("ML").getEntry("object_classes")

    print("Starting camera server")
    cs = CameraServer.getInstance()
    camera = UsbCamera('cam1', 0)
    camera.setResolution(WIDTH, HEIGHT)
    cvSink = cs.getVideo(camera=camera)
    img = np.zeros(shape=(HEIGHT, WIDTH, 3), dtype=np.uint8)
    output = cs.putVideo("MLOut", WIDTH, HEIGHT)

    print("Initializing ML engine")
    engine = DetectionEngine("model.tflite")
   # parser = PBTXTParser("map.pbtxt")
    #parser.parse()
    #labels = parser.get_labels()

    start = time()
    lower_yellow = np.array([20, 75, 100])
    upper_yellow = np.array([40, 255, 255])
    print("Starting ML mainloop")
    while True:
        t, frame = cvSink.grabFrame(img)

        # Run inference.
        ans = engine.detect_with_image(Image.fromarray(frame), threshold=0.5, keep_aspect_ratio=True, relative_coord=False, top_k=10)
        nb_objects_entry.setNumber(len(ans))

        boxes = []
        names = []

        # Display result.
        if ans:
            
            for obj in ans:
                box = [round(i, 3) for i in obj.bounding_box.flatten().tolist()]
                boxes.extend(box)
                xmin, ymin, xmax, ymax = map(int,box)
                print(xmin, ymin, xmax, ymax)

                label = '%s: %d%%' % ('power_cell', int(obj.score * 100))  # Example: 'Cargo: 72%'
                label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                label_ymin = max(ymin, label_size[1] + 10)
                
                cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10),
                              (xmin + label_size[0], label_ymin + base_line - 10), (255, 255, 255), cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 4)

        else:
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
            contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2] 
            largest_area = 0
            largest_contour = np.array([0, 0])
            for c in contours:
                area = cv2.contourArea(c)
                if area > largest_area:
                    largest_contour = c
                    largest_area = area
            if(largest_area > 0):
                x,y,w,h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame,(x,y),(x+w, y+h),(0,255,0),2)
        output.putFrame(frame)
        boxes_entry.setDoubleArray(boxes)
        object_classes_entry.setStringArray(names)
        #print("FPS: {:.1f}".format(1 / (time() - start)))

        start = time()
        


if __name__ == '__main__':
    config_file = "/boot/frc.json"
    main(config_file)