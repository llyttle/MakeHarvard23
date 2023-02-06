import cv2

min_dist_for_separate_face = 100
#opencv_xml_path_base = "/home/hvakil/.local/lib/python3.8/site-packages/cv2/data/"
#frontFaceCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_frontalface_default" + ".xml")
#sideFaceCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_profileface" + ".xml")
#upperBodyCascade = cv2.CascadeClassifier(opencv_xml_path_base + "haarcascade_upperbody" + ".xml")

red_lower_bound = 150
green_lower_bound = 40
blue_lower_bound = 0
red_upper_bound = 255
green_upper_bound = 100
blue_upper_bound = 50
h_lower_bound = 0
s_lower_bound = 0
v_lower_bound = 0
h_upper_bound = 255
s_upper_bound = 255
v_upper_bound = 50

### DECLARE VIEWED IMAGE CROPPING HERE
y_min = 0
y_max = 600
x_min = 160
x_max = 505
x_range = x_max - x_min
y_range = y_max - y_min
  

video_capture = cv2.VideoCapture("/dev/video6")

while True:
    #cv2.namedWindow('video_window')
    #cv2.namedWindow('image_info')

    cv2.namedWindow('hsv_window')

    # Capture frame-by-frame
    ret, frames = video_capture.read()
    #frames = cv2.resize(frames, (1500, 1000), interpolation=cv2.INTER_AREA)

    sky = frames[y_min:y_max, x_min:x_max]
    #print(f"width: {frames.get(3)}, height: {frames.get(4)}")
    cv2.imshow('Video2', sky)
    hsv_image = cv2.cvtColor(sky, cv2.COLOR_BGR2HSV)
    binary_image = cv2.inRange(sky, (blue_lower_bound,green_lower_bound,red_lower_bound), (blue_upper_bound,green_upper_bound,red_upper_bound))
    binary_hsv = cv2.inRange(hsv_image, (h_lower_bound, s_lower_bound, v_lower_bound), (h_upper_bound, s_upper_bound, v_upper_bound))

    contours, _ = cv2.findContours(binary_hsv, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    try:
        blob = max(contours, key=lambda el: cv2.contourArea(el))
        area = cv2.contourArea(blob)     
                    
        if area > 100: #done with wide angle lens, ball has area of ~200 at range of ~3m
            

            M = cv2.moments(blob)
            x_mid = (M["m10"] / M["m00"])
            y_mid = (M["m01"] / M["m00"])
            x_frac = x_mid / x_range
            y_frac = y_mid / y_range
            print(f"x mid: {x_mid}, y mid: {y_mid}")
            print(f"x frac: {x_frac}, y frac: {y_frac}\n")
            center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
            goal_image_x = int(M["m10"] / M["m00"])
            _,_,_,h = cv2.boundingRect(blob)
            
            canvas = binary_hsv.copy()
            cv2.circle(canvas, center, 20, (50,50,50), 5)
            cv2.imshow('masked_window', canvas)
            cv2.waitKey(5)
    except:
        pass
   

    cv2.imshow('rgb window', binary_image)
    cv2.imshow('hsv_window', binary_hsv)

    # Display the resulting frame
    #cv2.imshow('Video', frames)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

video_capture.release()
cv2.destroyAllWindows()


print("finished successfully")
