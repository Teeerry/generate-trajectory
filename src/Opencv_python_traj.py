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

# Use resize instaed
def im_res(im):
    # Keep the aspect ration of the image
    # Limit the workspace for drawing
    # The max workspace is 600mm x 600mm
    print("The image shape before resize: ",im.shape)
    x = im.shape[0]
    y = im.shape[1]
    y_max = 388.0
    x_max = 388.0
    fy = y_max/y
    fx = x_max/x
    
    if(y >= x):
        im_resize = cv2.resize(im, None, fx=fy, fy=fy, interpolation=cv2.INTER_CUBIC)
    else:
        im_resize = cv2.resize(im, None, fx=fx, fy=fx, interpolation=cv2.INTER_CUBIC)
    print('The image shape after resize: ' , im_resize.shape)
    return im_resize

im = cv2.imread('../data/opencv-logo.png')
cv2.imshow('src_im', im)
print("image shape: ",im.shape)
im = im_res(im)
#cv2.imshow('res_im', im)
print("image shape: ",im.shape)
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)


# 查找轮廓
ret, thresh = cv2.threshold(imgray, 244, 255, 0)
img, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

#过滤太小的contour
contours2=[cnt for cnt in contours if cv2.contourArea(cnt)>200]

# 打印轮廓信息
print('len(contours): ', len(contours))
print('len(contours2): ', len(contours2))

# 绘制轮廓
cv2.drawContours(im, contours, 1 , (255, 0, 0), 3)

# 显示图片
cv2.imshow('drawContours', im)

# 等待退出
if cv2.waitKey(0) & 0xff == ord('q'):
    cv2.destroyAllWindows()
