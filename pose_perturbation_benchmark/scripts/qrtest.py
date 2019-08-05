#!/usr/bin/env python
from __future__ import print_function
import pyzbar.pyzbar as pyzbar
import numpy as np
import cv2
 
def decode(im) : 
  # Find barcodes and QR codes
  decodedObjects = pyzbar.decode(im)
 
  # Print results
  for obj in decodedObjects:
    print('Type : ', obj.type)
    print('Data : ', obj.data,'\n')
     
  return decodedObjects
 
 
# Display barcode and QR code location  
def display(im, decodedObjects):
 
  # Loop over all decoded objects
  for decodedObject in decodedObjects: 
    points = decodedObject.polygon
 
    # If the points do not form a quad, find convex hull
    if len(points) > 4 : 
      hull = cv2.convexHull(np.array([point for point in points], dtype=np.float32))
      hull = list(map(tuple, np.squeeze(hull)))
    else : 
      hull = points;
     
    # Number of points in the convex hull
    n = len(hull)
 
    # Draw the convext hull
    for j in range(0,n):
      cv2.line(im, hull[j], hull[ (j+1) % n], (255,0,0), 3)
 
  # Display results 
  cv2.imshow("Results", im);
  cv2.waitKey(0);

def main_f():
  cam = cv2.VideoCapture(0)

  cv2.namedWindow("test")

  img_counter = 0

  # while True:
  #     ret, frame = cam.read()
  #     cv2.imshow("test", frame)
  #     if not ret:
  #         break
  #     k = cv2.waitKey(1)

  #     if k%256 == 27:
  #         # ESC pressed
  #         print("Escape hit, closing...")
  #         break
  #     elif k%256 == 32:
  #         # SPACE pressed
  #         img_name = "opencv_frame_{}.png".format(img_counter)
  #         cv2.imwrite(img_name, frame)
  #         print("{} written!".format(img_name))
  #         img_counter += 1
  im_number = 100
  for i in range(im_number):
    ret, frame = cam.read()
    cv2.imshow("test", frame)
    img_name = "opencv_frame_{}.png".format(img_counter)
    cv2.imwrite(img_name, frame)
    #print("{} written!".format(img_name))
    img_counter += 1

  cam.release()

  cv2.destroyAllWindows()
 
  # Read image
  for i in range(im_number):
    im = cv2.imread('opencv_frame_{}.png'.format(i))
    # 'default_qrcode.jpg'
   
    decodedObjects = decode(im)
    #display(im, decodedObjects)
    #print(decodedObjects[0][1])



    try:
      if decodedObjects[0][1] == 'QRCODE':
        print('Yes')
        #display(im, decodedObjects)
        return('yes')
        break
    except IndexError:
      pass
   
# Main 
if __name__ == '__main__':
 x=main_f()
 print("output: ",x)
