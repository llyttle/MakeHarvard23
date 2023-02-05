# import the opencv library
import cv2
import numpy as np
import sys
import imutils

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
      self.src_points = np.array([(99,25),(519,19),(92,454),(535,450)])

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
    mask1 = cv2.inRange(hsv, np.array([10, 170, 165]), np.array([20, 255, 200]))  # orange mask
    mask2 = cv2.inRange(hsv, np.array([110, 80, 100]), np.array([180, 255, 170]))  # blue mask

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

    cv2.imshow('Person view', cv2.flip(im_out,1))
      
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
    self.y_max = 400
    self.x_min = 160
    self.x_max = 505
    self.x_range = self.x_max - self.x_min
    self.y_range = self.y_max - self.y_min

    self.pos = None  

    self.video_capture = cv2.VideoCapture("/dev/video2")

  def update(self):
    #cv2.namedWindow('video_window')
    #cv2.namedWindow('image_info')

    # cv2.namedWindow('hsv_window')

    # Capture frame-by-frame
    ret, frames = self.video_capture.read()
    #frames = cv2.resize(frames, (1500, 1000), interpolation=cv2.INTER_AREA)

    sky = frames[self.y_min:self.y_max, self.x_min:self.x_max]
    #print(f"width: {frames.get(3)}, height: {frames.get(4)}")
    # cv2.imshow('Video2', sky)
    hsv_image = cv2.cvtColor(sky, cv2.COLOR_BGR2HSV)
    binary_image = cv2.inRange(sky, np.array([0,40,150]), np.array([50,100,255]))   # Color filter
    binary_hsv = cv2.inRange(hsv_image, np.array([0, 0, 0]), np.array([255, 255, 50]))  # hsv filter

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

  
  def spin(self):
    self.RT.update()
    self.PT.update()

    robot_pos = self.RT.pos
    person_pos = self.PT.pos

    try:
      target_vec = (person_pos - robot_pos)
      rel_theta = np.arctan2(target_vec[1],target_vec[0])

      print("X: ", target_vec[0])
      print("Y: ", target_vec[1])
      print("theta: ", self.RT.theta)
      print("relative theta", rel_theta-self.RT.theta)
    except:
      print("no robot_pos or person_pos")

if __name__ == "__main__":
  CC = CentralController()
  while(True):
    CC.spin()