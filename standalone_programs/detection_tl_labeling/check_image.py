import glob
import csv
import os
import sys
import cv2

f = open('ex1.csv', 'r', encoding='utf-8')
rd = csv.reader(f)
list_csv = []
for line in rd:
    list_csv.append(line)

img = cv2.imread("ex1.jpg", cv2.IMREAD_ANYCOLOR)
for line in list_csv:
    cv2.line(img, (int(line[1]), int(line[2])), (int(line[1]), int(line[2])), (255, 255, 255), 8)
    cv2.rectangle(img, (int(line[1]), int(line[2])), (int(line[1])+int(line[3]), int(line[2])+int(line[4])), (0, 0, 255), 2)
cv2.imshow("TL", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
f.close()