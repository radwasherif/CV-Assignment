import cv2 as cv
import numpy as np
from numpy import float64

image1 = cv.imread('A1I/Q1I1.png')
image1 = np.array(image1, dtype=float64)
w1 = image1.shape[1]
contrast = np.ones(image1[0].shape)
step = 3.0/w1
for i in range(w1):
    if i >= 2* w1/3:
        contrast[i] -= np.ones(3)*(i - 2 * w1 / 3) * step

for i in range(image1.shape[0]):
    image1[i] = np.multiply(image1[i], contrast)
image1 = (image1 * 3)

image1[image1 > 255] = 255
# image1 = np.array(image1, dtype=np.uint8)
# cv.imshow('Batman wassignment1el sett el monhara', image1)

image2 = cv.imread('A1I/Q1I2.jpg')
image2 = np.array(image2, dtype=float64)
image2 = cv.flip(image2, 1)
fy = image1.shape[0]*1.0/image2.shape[0]
fx = image2.shape[1]/ image2.shape[0] * fy
image2 = cv.resize(image2, (0, 0), fx = fx, fy= fy)



left = image1.shape[1] - image2.shape[1]
print(image1.shape)
print(image2.shape)

image2 = cv.copyMakeBorder(image2, 0, 0, left, 0, cv.BORDER_DEFAULT, value=[0, 0, 0])
q1 = image1 + image2
q1[q1 > 255] = 255
q1 = np.array(q1, dtype=np.uint8)
cv.imshow('Batman wel sett el monhara', q1)

cv.waitKey()


#############
      # QUESTTION 2 - Part I
#############
min_x, min_y = 1219, 378
max_x, max_y = 1308, 517

room = cv.imread('A1I/Q2I2.jpg')
ben = cv.imread('A1I/Q2I1.jpg')

room_w, room_h = room.shape[1], room.shape[0]
ben_w, ben_h = ben.shape[1], ben.shape[0]
p1 = np.float32([[0, 0], [ben_w, 0], [0, ben_h]])
p2 = np.float32([[min_x, min_y], [max_x, min_y], [min_x, max_y]])
M = cv.getAffineTransform(p1, p2)
ben_pers = cv.warpAffine(ben, M, (room_w, room_h))
print(ben_pers.shape)
ben_mask = np.ones(ben_pers.shape, dtype=np.uint8) * 255
ben_mask[min_y:max_y, min_x:max_x] = [0, 0, 0]
room_mask = cv.bitwise_and(room, ben_mask)
room_result = room_mask + ben_pers
cv.imshow('Frame', room_result)
cv.waitKey()

#############
      # QUESTTION 2 - Part II
#############
lady = cv.imread('A1I/Q2I3.jpg')
ben = cv.imread('A1I/Q2I1.jpg')

lady_w, lady_h = lady.shape[1], lady.shape[0]
ben_w, ben_h = ben.shape[1], ben.shape[0]

p1, p2, p3, p4 = [373, 95], [704, 130], [664, 559], [327, 526]
pts1 = np.float32([[0, 0], [ben_w, 0], [ben_w, ben_h]])
pts2 = np.float32([p1, p2, p3])
M = cv.getAffineTransform(pts1, pts2)
ben_pers = cv.warpAffine(ben, M, (lady_w, lady_h))


ben_mask = np.ones(ben_pers.shape, dtype=np.uint8) * 255
poly_pts = np.array([[p1, p2, p3, p4]],dtype=np.int32)
ben_mask = cv.fillPoly(ben_mask, poly_pts, 0)
lady_mask = cv.bitwise_and(lady, ben_mask)
lady_result = lady_mask + ben_pers
cv.imshow('Frame2', lady_result)
cv.waitKey()


#############
      # QUESTTION 3
#############
frame = cv.imread('A1I/Q3I1.jpg')
frame_w, frame_h = frame.shape[1], frame.shape[0]
p1, p2, p3, p4 = [164, 37], [468, 71], [464, 353], [159, 389]
pts1 = np.float32([[0, 0], [ben_w, 0], [ben_w, ben_h], [0, ben_h]])
pts2 = np.float32([p1, p2, p3, p4])
M = cv.getPerspectiveTransform(pts1, pts2)
ben_pers = cv.warpPerspective(ben, M, (frame_w, frame_h))
ben_mask = np.ones(ben_pers.shape, dtype=np.uint8) * 255
poly_pts = np.array([[p1, p2, p3, p4]],dtype=np.int32)
ben_mask = cv.fillPoly(ben_mask, poly_pts, 0)
frame_mask = cv.bitwise_and(frame, ben_mask)
frame_result = frame_mask + ben_pers
cv.imshow('Frame3', frame_result)
cv.waitKey()