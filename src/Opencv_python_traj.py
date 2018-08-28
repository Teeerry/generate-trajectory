# -*- coding: utf-8 -*-
# @Time    : 2018/8/29 下午7:49
# @Author  : Terry
# @email   : terryluohello@qq.com


"""
函数 cv2.drawContours() 可以用来绘制轮廓。它可以根据你提供的界点绘制任何形状。
第一个参数是原始图像；
第二个参数是轮廓，一个Python列表；
第三个参数是轮廓的索引；
在绘制独立轮廓是很有用当设置为 -1 时绘制所有轮廓；
接下来的参数是轮廓的颜色和厚度等。
"""

import numpy as np
import cv2

im = cv2.imread('../data/opencv-logo.png')
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)

# print("image shape: ",im.shape)
# cv2.imshow('imgray', imgray)

# 查找轮廓
ret, thresh = cv2.threshold(imgray, 244, 255, 0)
img, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#过滤太小的contour
contours2=[cnt for cnt in contours if cv2.contourArea(cnt)>200]

# 打印轮廓信息
print('len(contours): ', len(contours))
print('len(contours2): ', len(contours2))
print("contours2 type: ", type(contours2))
print('len(contours2[9]): ', len(contours2[9]))
print("contours2 shape: ", np.array(contours2).shape)

#降到二维，[x,y]形式，并打印xy
print(contours2[9].flatten().reshape(len(contours2[9]),2)[0])

# 绘制轮廓
cv2.drawContours(imgray, contours2, -1, (0, 0,255), 3)
cv2.drawContours(im, contours2, 9 , (255, 0, 0), 3)

# 显示图片
cv2.imshow('drawContours', im)
cv2.imshow('drawContours-', imgray)

# 等待退出
cv2.waitKey(0)
cv2.destroyAllWindows()
