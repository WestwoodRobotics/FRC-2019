from networktables import NetworkTables
import numpy as np
import click
import cv2

from time import perf_counter
import collections
import math

def get_color_mask(image):
  hsv = cv2.cvtColor(input, cv2.COLOR_BGR2HSV)
  cv2.medianBlur(hsv, 5, hsv)
  
  halfW = hsv.shape[1] // 2
  maskL = cv2.inRange(hsv[:, :halfW], minColor[0], maxColor[0])
  maskR = cv2.inRange(hsv[:, halfW:], minColor[1], maxColor[1])
  mask = np.hstack((maskL, maskR))
    
  return mask, hsv 

def find_image_location(method, field_image, template):
  grey_field_image = cv2.cvtColor(field_image, cv2.COLOR_BGR2GRAY) # !! 
  res = cv2.matchTemplate(grey_field_image, template, method)

  min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(res)

  w, h = template.shape[::-1] 
  # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
  if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
      top_left = min_loc
  else:
      top_left = max_loc
  bottom_right = (top_left[0] + w, top_left[1] + h)

  rect_img = cv2.rectangle(grey_field_image, top_left, bottom_right, 255, 2)

  cv2.imshow("HELLO", res)
  cv2.imshow("Found Image", rect_img)

  # return cv2.minAreaRect(res)

def find_hatch(method, field_image, hatch_image="hatch.jpg"):
  return find_image_location(method, field_image, cv2.imread(hatch_image, 0))

def find_hatch_placement(method, field_image, hatch_placement_image="hatch_location.jpg"):
  return find_image_location(method, field_image, cv2.imread(hatch_placement_image, 0))

def find_hamaz(method, field_image, hamaz="hamaz.png"):
  return find_image_location(method, field_image, cv2.imread(hamaz, 0))

def find_hatch_distance():
  pass

def run(smartDashboard, cap):  
  KNOWN_DISTANCE = 24.0
  KNOWN_WIDTH = 11.0
  METHOD = cv2.TM_CCOEFF_NORMED

  while cap.isOpened():
    ret, frame = cap.read()
    print(frame)
    cv2.imshow("preview", frame)
    cv2.waitKey()

    find_hamaz(METHOD, field_image=frame)

def init_image_capture(device):
  cap = cv2.VideoCapture(device)
  if not cap.isOpened():
    print("failed to open camera device {0}".format(device))
    if device != 0:
      print("resetting device id to 0")
      return init_image_capture(0)

  cap.set(cv2.CAP_PROP_FRAME_WIDTH, 600)
  cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 600)
  print("done initializing the image capture")

  return cap

@click.command()
@click.option("--ip", default="10.2.54.2", help="Default ip to connect to")
@click.option("--network", default=True, help="Whether to connect with a network")
@click.option("--device", default=0, help="Device id used to connect to")
def main(ip, network, device):
  smartDashboard = None
  if network:
    NetworkTables.initialize(server=ip)
    smartDashboard = NetworkTables.getTable("SmartDashboard")

  cap = init_image_capture(device)
  run(smartDashboard, cap)

if __name__ == '__main__':
  main()
