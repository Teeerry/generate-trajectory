import cv2
import numpy as np

img = cv2.imread('../data/opencv-logo.png')
imgray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
# 查找轮廓
ret, thresh = cv2.threshold(imgray, 244, 255, 0)
img, contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
#过滤太小的contour
contours2=[cnt for cnt in contours if cv2.contourArea(cnt)>200]
# 打印轮廓信息
print('len(contours): ', len(contours))
print('len(contours2): ', len(contours2))
# 绘制轮廓
cv2.drawContours(img, contours, -1 , (255, 0, 0), 3)
mask = np.ones(img.shape[:2], dtype="uint8") * 255
# Draw the contours on the mask
cv2.drawContours(mask, contours, -1, 0, -1)
# remove the contours from the image and show the resulting images
image = cv2.bitwise_and(img, img, mask=mask)
cv2.imshow("Source", img)
cv2.imshow("Mask", mask)
cv2.imshow("After", image)


# thin algorithm
size = np.size(image)
skel = np.zeros(image.shape,np.uint8)
print("Size",size)
print("Thin before",cv2.countNonZero(image))
ret,image = cv2.threshold(image,127,255,0)
element = cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
done = False
while(not done):
    eroded = cv2.erode(image,element)
    temp = cv2.dilate(eroded,element)
    temp = cv2.subtract(image,temp)
    skel = cv2.bitwise_or(skel,temp)
    image = eroded.copy()
 
    zeros = size - cv2.countNonZero(image)
    if zeros==size:
        done = True
print("Thin after",cv2.countNonZero(skel))
cv2.imshow("skel",skel)

# 等待退出
if cv2.waitKey(0) & 0xff == ord('q'):
    cv2.destroyAllWindows()
