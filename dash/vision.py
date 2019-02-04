from networktables import NetworkTables
import numpy as np
import click
import cv2

from time import perf_counter
import collections
import math

def find_hatch(method, field_image, hatch_image):
  # Apply template Matching
  res = cv.matchTemplate(field_image, hatch_image, method)
  min_val, max_val, min_loc, max_loc = cv.minMaxLoc(res)
  # If the method is TM_SQDIFF or TM_SQDIFF_NORMED, take minimum
  if method in [cv.TM_SQDIFF, cv.TM_SQDIFF_NORMED]:
    top_left = min_loc
  else:
    top_left = max_loc

  bottom_right = (top_left[0] + w, top_left[1] + h)
  res = cv.rectangle(img, top_left, bottom_right, 255, 2)
  print(res)
  
  return cv2.minAreaRect(res)


def find_hatch_distance():

def run(smartDashboard):
  while True: 


@click.command()
@click.option("--ip", default="10.2.54.2", help="Default ip to connect to")
@click.option("")
def main(ip):
  NetworkTables.initialize(server=ip)
  smartDashboard = NetworkTables.getTable("SmartDashboard")

  KNOWN_DISTANCE = 24.0
  KNOWN_WIDTH = 11.0

  image = cv2.imread("images/2ft.png")
  marker = find_marker(image)
  focalLength = (marker[1][0] * KNOWN_DISTANCE) / KNOWN_WIDTH

if __name__ == '__main__':
  main()
