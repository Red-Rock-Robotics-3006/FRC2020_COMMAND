import numpy as np
import math
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

def listener(table, key, value, isNew):
    print("value changed: key: '%s'; value: %s; isNew: %s" % (key, value, isNew))
    return value
def connectionListener(connected, info):
    print(info, "; Connected=%s" % connected)

def distToTape():
    pass

def main(config):
    
    team = read_config(config)
    WIDTH, HEIGHT = 320, 240
    FOV = 68.5
    focal_length = WIDTH / (2*math.tan(math.radians(FOV/2)))
    print("Starting camera server")
    
    cs = CameraServer.getInstance()
    power_cell_cam = UsbCamera('power_cell_cam', 2)
    power_cell_cam.setResolution(WIDTH, HEIGHT)
    power_cell_cam.setExposureManual(25)

    tape_cam = UsbCamera('tape_cam', 0)
    tape_cam.setResolution(WIDTH, HEIGHT)
    tape_cam.setExposureManual(1)

    print("Connecting to Network Tables")
    ntinst = NetworkTablesInstance.getDefault()
    ntinst.startClientTeam(team)
    ntinst.addConnectionListener(connectionListener, immediateNotify = True)

    """Format of these entries found in WPILib documentation."""
    power_cell_angle = ntinst.getTable('ML').getEntry('power_cell_angle')
    power_cell_pos = ntinst.getTable('ML').getEntry('power_cell_pos')
    power_cell_exists = ntinst.getTable('ML').getEntry('power_cell_exists')
    power_cell_dist = ntinst.getTable('ML').getEntry('power_cell_dist')
    power_cell_x = ntinst.getTable('ML').getEntry('power_cell_x')
    power_cell_y = ntinst.getTable('ML').getEntry('power_cell_y')


    tape_angle = ntinst.getTable('ML').getEntry('tape_angle')
    tape_dist = ntinst.getTable('ML').getEntry('tape_dist')
    tape_found = ntinst.getTable('ML').getEntry('tape_found')
    sd = ntinst.getTable('SmartDashboard')

    sd.addEntryListener(listener, key='cam')

    cvSink = cs.getVideo(camera=power_cell_cam)
    img = np.zeros(shape=(HEIGHT, WIDTH, 3), dtype=np.uint8)
    output = cs.putVideo("MLOut", WIDTH, HEIGHT)

    print("Initializing ML engine")
    engine = DetectionEngine("model.tflite")

    lower_yellow = np.array([20, 75, 100])
    upper_yellow = np.array([40, 255, 255])

    lower_green = np.array([55, 124, 81]) #137 240 135 - HSV, 0,90,90 - RGB
    upper_green = np.array([255, 255, 255]) #143 255 148 - HSV, 86,255,255 - RGB

    lower_color = np.array([0, 0, 0])
    upper_color = np.array([0, 0, 0])

    camera_robot_x = .3302
    camera_robot_y = -.0254

    print("Starting ML mainloop")
    
    while True:
        cam_mode = sd.getBoolean(key='cam', defaultValue=False)

        if cam_mode:
            cvSink.setSource(tape_cam)
            lower_color = lower_green
            upper_color = upper_green
            #print('green')
        else:
            cvSink.setSource(power_cell_cam)
            lower_color = lower_yellow
            upper_color = upper_yellow
            #print('yellow')

        t, frame = cvSink.grabFrame(img)

        # Run inference.

        if not cam_mode:
            ans = engine.detect_with_image(Image.fromarray(frame), threshold=0.5, keep_aspect_ratio=True, relative_coord=False, top_k=10)
        else:
            ans = False
       
        center_power_cell_x = WIDTH/2
        center_power_cell_y = HEIGHT/2

        bool_power_cell = False
        bool_tape = False

        largest_box_xmin = 0
        largest_box_xmax = 0
        largest_box_ymin = 0
        largest_box_ymin = 0
        largest_box_area = 0

        minimum_box_area = 0

        angle_to_turn_power_cell = 0
        angle_to_turn_power_cell_robot = 0
        dist_power_cell = 0

        angle_to_turn_tape = 0
        dist_tape = 0

        new_x = 0
        new_y = 0

        center_contour_x = WIDTH/2

        # Display result.
        if ans:
            for obj in ans:
                box = [round(i, 3) for i in obj.bounding_box.flatten().tolist()]
                xmin, ymin, xmax, ymax = map(int,box)
                box_area = (xmax - xmin) * (ymax - ymin)

                if box_area > largest_box_area:
                    largest_box_area = box_area
                    largest_box_xmin = xmin
                    largest_box_xmax = xmax
                    largest_box_ymin = ymin
                    largest_box_ymax = ymax

                label = '%s: %d%%' % ('power_cell', int(obj.score * 100))  # Example: 'Cargo: 72%'
                label_size, base_line = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2)
                label_ymin = max(ymin, label_size[1] + 10)
                
                cv2.rectangle(frame, (xmin, label_ymin - label_size[1] - 10),
                              (xmin + label_size[0], label_ymin + base_line - 10), (255, 255, 255), cv2.FILLED)
                cv2.putText(frame, label, (xmin, label_ymin - 7), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
                cv2.rectangle(frame, (xmin, ymin), (xmax, ymax), (10, 255, 0), 4)
            
            if(largest_box_area > minimum_box_area):
                center_power_cell_x = 0.5 * (largest_box_xmin + largest_box_xmax)
                center_power_cell_y = 0.5 * (largest_box_ymin + largest_box_ymax)
                bool_power_cell = True
                
                angle_to_turn_power_cell = math.degrees(math.atan((center_power_cell_x -(.5*WIDTH - .5))/focal_length))
                angle_to_right_side = math.degrees(math.atan((largest_box_xmax -(.5*WIDTH - .5))/focal_length))
                angle_diff_right_center = abs(angle_to_right_side - angle_to_turn_power_cell)
                dist_power_cell = (.5842/(2*math.pi))/math.tan(math.radians(angle_diff_right_center))
                
                x_vec = dist_power_cell / math.sqrt(1 + math.tan(math.radians(-angle_to_turn_power_cell))**2)
                y_vec = x_vec * math.tan(math.radians((-angle_to_turn_power_cell)))
                if (angle_to_turn_power_cell < 0):
                    x_vec *= -1
                    y_vec *= -1
                new_x = camera_robot_x + x_vec
                new_y = camera_robot_y + y_vec
                angle_to_turn_power_cell_robot = - math.degrees(math.atan2(new_y,new_x))

                cv2.putText(frame, str(new_x), (100, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(new_y), (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(dist_power_cell), (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                
        else:

            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, lower_color, upper_color)
            contours = cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)[-2] 

            '''
            TODO: Determine how reflective tapes will look through eye of god
            TODO: Calculate angle and distance to tape (tape angle calculation same as power cell angle calculation)
            '''

            largest_contour = np.array([0, 0])
            for c in contours:
                area = cv2.contourArea(c)
                if area > largest_box_area:
                    largest_contour = c
                    largest_box_area = area

            if(largest_box_area > minimum_box_area and not cam_mode):
                x,y,w,h = cv2.boundingRect(largest_contour)
                cv2.rectangle(frame,(x,y),(x+w, y+h),(0,255,0),2)
                center_power_cell_x = 0.5 * (x + x + w)
                center_power_cell_y = 0.5 * (y + y + h) 
                bool_power_cell = True
                
                angle_to_turn_power_cell = math.degrees(math.atan((center_power_cell_x -(.5*WIDTH - .5))/focal_length))
                angle_to_right_side = math.degrees(math.atan(((x+w) -(.5*WIDTH - .5))/focal_length))
                angle_diff_right_center = angle_to_right_side - angle_to_turn_power_cell
                dist_power_cell = (.5842/(2*math.pi))/math.tan(math.radians(angle_diff_right_center))
                
                x_vec = dist_power_cell / math.sqrt(1 + math.tan(math.radians(-angle_to_turn_power_cell))**2)
                y_vec = x_vec * math.tan(math.radians((-angle_to_turn_power_cell)))
                if(angle_to_turn_power_cell < 0):
                    x_vec *= -1
                    y_vec *= -1
                new_x = camera_robot_x + x_vec
                new_y = camera_robot_y + y_vec
                angle_to_turn_power_cell_robot = - math.degrees(math.atan2(new_y,new_x))
                
                cv2.putText(frame, str(new_x), (100, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(new_y), (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                cv2.putText(frame, str(dist_power_cell), (100, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            elif (largest_box_area > 0 and cam_mode):
                x,y,w,h = cv2.boundingRect(largest_contour)
                M1 = cv2.moments(largest_contour)
                center_contour_x = int(M1['m10']/M1['m00'])
                cv2.drawContours(frame, [largest_contour], 0, (0, 255, 0), 3)
                
                angle_to_turn_tape = math.degrees(math.atan((center_contour_x -(.5*WIDTH - .5))/focal_length))
                angle_to_right_side = math.degrees(math.atan(((x+w) -(.5*WIDTH - .5))/focal_length))
                angle_diff_right_center = abs(angle_to_right_side - angle_to_turn_tape)
                dist_tape = .99695/math.tan(math.radians(angle_diff_right_center))

                cv2.putText(frame, str(dist_tape), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                bool_tape = True

        output.putFrame(frame)

        tape_found.setBoolean(bool_tape)

        if cam_mode:
            angle_to_turn_tape = (center_contour_x / WIDTH * FOV) - FOV/2
            tape_angle.setDouble(angle_to_turn_tape)
            #TODO: Send distance to tape
        else:
            power_cell_angle.setDouble(angle_to_turn_power_cell_robot)
            power_cell_pos.setDoubleArray([center_power_cell_x, center_power_cell_y])
            power_cell_exists.setBoolean(bool_power_cell)
            power_cell_dist.setDouble(dist_power_cell)
            power_cell_x.setDouble(new_x)
            power_cell_y.setDouble(new_y)

            #print(angle_to_turn_power_cell)

if __name__ == '__main__':
    config_file = "/boot/frc.json"
    main(config_file)
