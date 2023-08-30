#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2

img = cv2.imread('/home/pilz/Pictures/smart_factory/cap_calibrated.png')  # 直接读为灰度图像
x, y = img.shape[0:2]

reshaped_img = cv2.resize(img, (int(y / 1), int(x / 1)))
cv2.imwrite('/home/pilz/Pictures/smart_factory/cap_calibrated_reshaped.png', reshaped_img)

# BGR转化为HSV
HSV = cv2.cvtColor(reshaped_img, cv2.COLOR_BGR2HSV)


# 鼠标点击响应事件
def getposHsv(event, x, y, flags,  param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("HSV is", HSV[y, x])


def getposBgr(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print("Bgr is", reshaped_img[y, x])


cv2.imshow("imageHSV", HSV)
cv2.imshow('image', reshaped_img)
cv2.setMouseCallback("imageHSV", getposHsv)
cv2.setMouseCallback("image", getposBgr)
cv2.waitKey(0)
