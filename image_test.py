# import the opencv library
import cv2
import numpy as np
import sys
import imutils
import serial
import serial.tools.list_ports as list_ports
import time,rclpy
import math




class Serial_cmd:
    Arduino_IDs = ((0x2341, 0x0043), (0x2341, 0x0001), 
                   (0x2A03, 0x0043), (0x2341, 0x0243), 
                   (0x0403, 0x6001), (0x1A86, 0x7523),
                   (0x9025, 0x0067))
    
    def __init__(self, port=''):
        print("test abc")
        if port == '':
            self.dev = None
            self.connected = False
            devices = list_ports.comports()
            for device in devices:
                print(device)
                print(type(device))
                print(device.vid)
                print(type(device.vid))
                if device.vid == 9025:
                    print("before try")
                    try:
                        self.dev = serial.Serial(device.device, 115200)
                        self.connected = True
                        print('Connected to {!s}...'.format(device.device))
                    except:
                        print("an exception occurred :(")
                if self.connected:
                    break
        else:
            try:
                self.dev = serial.Serial(port, 115200)
                self.connected = True
            except:
                self.edev = None
                self.connected = False
    def write_data_to_arduino(self, string_to_write):
        if self.connected:
            self.dev.write(string_to_write.encode())
        else:
            raise Exception("self not connected!")
    def read_data(self):
        if self.connected:
            try:
                print(self.dev.readline().decode().rstrip())
            except:
                pass

class robotTracker():
  def __init__(self):
    # define a video capture object
    self.vid = cv2.VideoCapture("/dev/video4")

    self.src_points = []
    self.dst_size = 480
    self.dst_points = np.array([[0,0],[self.dst_size,0],[0,self.dst_size],[self.dst_size,self.dst_size]])

    self.pos = None
    self.theta = None

    self.recalibrate_homography(True)

  def recalibrate_homography(self, active):
    #top left, top right, bottom left, bottom right
    if active:
      while len(self.src_points) < 4:
        ret, frame = self.vid.read()

        cv2.namedWindow('Select Table Boundaries')

        cv2.setMouseCallback('Select Table Boundaries',self.mouseSelect)

        for point in self.src_points:
          frame = cv2.circle(frame, point, radius=2, color=(0, 0, 255), thickness=-1)

        cv2.imshow('Select Table Boundaries', frame)

        self.windowCheck()

      cv2.destroyWindow('Select Table Boundaries')

      self.src_points = np.array(self.src_points)

      print(self.src_points)

    else:
      self.src_points = np.array([(130,53),(370,53),(130,360),(370,360)])

    self.h, status = cv2.findHomography(self.src_points, self.dst_points)

  def filter_small(self, img):
    cnts = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cnts = imutils.grab_contours(cnts)
    cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
    rect_areas = []
    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        rect_areas.append(w * h)
    avg_area = np.mean(rect_areas)
    for c in cnts:
        (x, y, w, h) = cv2.boundingRect(c)
        cnt_area = w * h
        if cnt_area < 0.5 * avg_area:
            img[y:y + h, x:x + w] = 0
    return img

  def locate_color(self, mask):
    mask = self.filter_small(mask)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    M = cv2.moments(contours[0])
    x_centroid = round(M['m10'] / M['m00'])
    y_centroid = round(M['m01'] / M['m00'])
    # cv2.circle(mask, (round(M['m10'] / M['m00']), round(M['m01'] / M['m00'])), 5, (0, 255, 0), -1)

    return np.array([x_centroid, y_centroid])

  def transform_correct(self):
    self.pos[1] = self.dst_size - self.pos[1]
    self.pos /= self.dst_size
    self.theta *= -1

  def update(self):
    ret, frame = self.vid.read()

    # self.viewer.namedWindow('Robot Space')
      
    im_out = cv2.warpPerspective(frame, self.h, (self.dst_size,self.dst_size))

    hsv = cv2.cvtColor(im_out, cv2.COLOR_BGR2HSV)

    # prepare the mask to overlay
    mask1 = cv2.inRange(hsv, np.array([5, 220, 150]), np.array([15, 255, 200]))  # orange mask
    mask2 = cv2.inRange(hsv, np.array([100, 220, 100]), np.array([120, 255, 170]))  # blue mask

    try:
      front = self.locate_color(mask1)
      back = self.locate_color(mask2)

      im_out = cv2.arrowedLine(im_out, back, front, 
                      (0,255,0), 1, tipLength = 0.5)

      self.pos = (front+back)/2
      vec = front-back
      self.theta = np.arctan2(vec[1], vec[0])

      self.transform_correct()

    except:
      self.pos = None
      self.theta = None

    # print("Position: ", self.pos)
    # print("Theta: ", self.theta)

    cv2.imshow('Robot view', cv2.flip(im_out,1))
    #cv2.imshow('orange', cv2.flip(mask1,1))
    #cv2.imshow('blue', cv2.flip(mask2,1))
    self.windowCheck()

  def mouseSelect(self, event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
      self.src_points.append((x,y))

  def windowCheck(self):
    if cv2.waitKey(1) & 0xFF == ord('q'):
      self.vid.release()
      cv2.destroyAllWindows()



class personTracker():
  def __init__(self):
    min_dist_for_separate_face = 100
    #opencv_xml_path_base = "/home/hvakil/.local/lib/python3.8/site-packages/cv2/data/"
    #frontFaceCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_frontalface_default" + ".xml")
    #sideFaceCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_profileface" + ".xml")
    #upperBodyCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_upperbody" + ".xml")

    ### DECLARE VIEWED IMAGE CROPPING HERE
    self.y_min = 0
    self.y_max = 600
    self.x_min = 100
    self.x_max = 560
    self.x_range = self.x_max - self.x_min
    self.y_range = self.y_max - self.y_min

    self.pos = None  

    self.video_capture = cv2.VideoCapture("/dev/video6")

  def update(self):
    #cv2.namedWindow('video_window')
    #cv2.namedWindow('image_info')

    # cv2.namedWindow('hsv_window')

    # Capture frame-by-frame
    ret, frames = self.video_capture.read()
    #frames = cv2.resize(frames, (1500, 1000), interpolation=cv2.INTER_AREA)

    sky = frames[self.y_min:self.y_max, self.x_min:self.x_max]
    #print(f"width: {frames.get(3)}, height: {frames.get(4)}")
    cv2.imshow('Video2', sky)
    hsv_image = cv2.cvtColor(sky, cv2.COLOR_BGR2HSV)
    binary_hsv = cv2.inRange(hsv_image, np.array([0, 0, 0]), np.array([255, 255, 40]))  # hsv filter

    contours, _ = cv2.findContours(binary_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    try:
      blob = max(contours, key=lambda el: cv2.contourArea(el))
      area = cv2.contourArea(blob)    
                  
      if area > 100: #done with wide angle lens, ball has area of ~200 at range of ~3m
        M = cv2.moments(blob)
        x_mid = (M["m10"] / M["m00"])
        y_mid = (M["m01"] / M["m00"])
        x_frac = x_mid / self.x_range
        y_frac = y_mid / self.y_range
        # print(f"x mid: {x_mid}, y mid: {y_mid}")
        # print(f"x frac: {x_frac}, y frac: {y_frac}\n")
        center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
        goal_image_x = int(M["m10"] / M["m00"])

        self.pos = np.array([x_frac, y_frac])

        canvas = binary_hsv.copy()
        cv2.circle(canvas, center, 20, (50,50,50), 5)
        cv2.imshow('masked_window', cv2.flip(canvas, 1))
        cv2.waitKey(5)

    except:
      self.pos = None
   
    # cv2.imshow('rgb window', binary_image)
    # cv2.imshow('hsv_window', binary_hsv)

    # Display the resulting frame
    # cv2.imshow('Video', frames)
    
    self.windowCheck()

  def windowCheck(self):
    if cv2.waitKey(1) & 0xFF == ord('q'):
      self.video_capture.release()
      cv2.destroyAllWindows()
      print("finished successfully")

class CentralController():
  def __init__(self):
    self.RT = robotTracker()
    self.PT = personTracker()
    self.writer = Serial_cmd()
    self.theta_threshold = math.pi / 4
    self.print_debugging = True
    print("initializing serial connection")
    if not self.writer.connected:
        raise Exception("error connecting")

  
  def spin(self):
    self.RT.update()
    self.PT.update()
    rel_theta = 0

    robot_pos = self.RT.pos
    person_pos = self.PT.pos

    try:
        target_vec = (person_pos - robot_pos)
        dist_to_target = math.sqrt(target_vec[0] ** 2 + target_vec[1] ** 2)
        target_theta = np.arctan2(target_vec[1],target_vec[0])# - self.RT.theta
        rel_theta = target_theta - self.RT.theta
        if rel_theta > math.pi:
            print("big rel theta")
            rel_theta = rel_theta - 2 * math.pi
        elif rel_theta < -1 * math.pi:
            print("small rel theta")
            rel_theta = rel_theta + 2 * math.pi


        print("X: ", target_vec[0])
        print("Y: ", target_vec[1])
        print("theta: ", self.RT.theta)
        print("relative theta", rel_theta)
    except:
      print("no robot_pos or person_pos")
    
    if abs(rel_theta) < 0.01:
        dir = "stop"
    elif rel_theta > 0:
        dir = "left"
    elif abs(rel_theta) < self.theta_threshold:
        dir = "straight"
    else:
        dir = "right"
    try:
        if dist_to_target < 0.1:
            dir = "stop"
    except:
        pass
    if self.print_debugging:
        print(f"Direction: {dir}")
    mod = 0
    match dir:
        case "left":
            left_motor_speed = 0
            right_motor_speed = 120 + mod
        case "right":
            left_motor_speed = 120 + mod
            right_motor_speed = 0
        case "straight":
            left_motor_speed = 130 + mod
            right_motor_speed = 130 + mod
        case "stop":
            left_motor_speed = 0
            right_motor_speed = 0
        case _:
            raise Exception("Invalid direction.")
    to_print = ""
    string_to_write = f"<{left_motor_speed},{right_motor_speed}\n"
    self.writer.write_data_to_arduino(string_to_write)
    to_print += string_to_write
    if self.print_debugging:
        print(to_print)
    while self.writer.dev.in_waiting:
        self.writer.read_data()
    time.sleep(0.08)

if __name__ == "__main__":
  CC = CentralController()
  while(True):
    CC.spin()
