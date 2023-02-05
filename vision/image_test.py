# import the opencv library
import cv2
import numpy as np
import sys
import imutils

class robotTracker():
  def __init__(self):
    # define a video capture object
    self.vid = cv2.VideoCapture("/dev/video2")

    self.src_points = []
    self.dst_size = 480
    self.dst_points = np.array([[0,0],[self.dst_size,0],[0,self.dst_size],[self.dst_size,self.dst_size]])

    self.pos = None
    self.theta = None

    self.recalibrate_homography(False)

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
      self.src_points = np.array([(114,56),(525,49),(101,477),(551,479)])

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

    cv2.namedWindow('Robot Space')
      
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

    print("Position: ", self.pos)
    print("Theta: ", self.theta)

    cv2.imshow('raw', im_out)
      
    self.windowCheck()

  def mouseSelect(self, event,x,y,flags,param):
    if event == cv2.EVENT_LBUTTONDOWN: #checks mouse left button down condition
      self.src_points.append((x,y))

  def windowCheck(self):
    if cv2.waitKey(1) & 0xFF == ord('q'):
      self.vid.release()
      cv2.destroyAllWindows()

if __name__ == '__main__':
  controller = robotTracker()
  while(True):
    controller.update()