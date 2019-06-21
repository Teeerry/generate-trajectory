# -*- coding: utf-8 -*-
#! /usr/bin/env python
import rospy
import time
import actionlib
import cv2
import numpy as np
import collections
from generate_traj.msg import TrajAction, TrajGoal, TrajResult, TrajFeedback
# Define the feedback function
def feedback_cb(feedback):
    print('[Feedback]:')
    print(feedback.sequence)

# Initialize the client
rospy.init_node('traj_action_client')
client = actionlib.SimpleActionClient('traj_action', TrajAction)
print("Waiting for server......")
client.wait_for_server()
print("Connected to server......")
##############################################################################
'''
Global variable definition
'''
kernel_size = 3
LowThreshold = 20
ratio = 3

'''
Function definition
'''
def flatten(lst):
    result = []
    def fly(lst):
        for item in lst:
            if isinstance(item, collections.Iterable) and not isinstance(item, (str, bytes)):
                fly(item)
            else:
                result.append(item)
    fly(lst)
    return result


'''
Image processing part
'''
rgb_img = cv2.imread("sixty_two_cut.jpg")
gray = cv2.cvtColor(rgb_img, cv2.COLOR_BGR2GRAY)

gray_filter = cv2.GaussianBlur(gray, (7, 7), 0)
_, nor_thresh = cv2.threshold(gray_filter, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
detect_edges_nor = cv2.Canny(nor_thresh, LowThreshold, LowThreshold * ratio, apertureSize = kernel_size)

# findcontour
image_nor, contours_nor, hierarchy_nor = cv2.findContours(nor_thresh, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
erosion_kernel = np.ones((3, 3), np.uint8)

# 待会用来存放每一段边缘的列表
contour_lst1 = []
contour_lst2 = []
contour_lst3 = []

# Extract each contour separately
# 如果需要泛化，这里contour层数需要随不同图片而改变
for i in [0, 1, 2]:
    contours_dst = np.zeros((gray.shape[0], gray.shape[1]), np.uint8)
    cv2.drawContours(contours_dst, contours_nor, i, (255, 255, 0), 2)
    image_nor_filter = cv2.GaussianBlur(contours_dst, (3, 3), 0)
    _, filter_thresh = cv2.threshold(image_nor_filter, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
   
    # 轮廓变粗了，需要腐蚀操作
    erosion = cv2.erode(filter_thresh, erosion_kernel, iterations = 1)
    detect_edges_filter = cv2.Canny(erosion, LowThreshold, LowThreshold * ratio, apertureSize = kernel_size)
    image_final, contours_final, _= cv2.findContours(erosion, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    image_final = cv2.drawContours(image_final, contours_final, 0, (155, 100, 100), 1)
    
    if (i == 0):
        contour_lst1 = contours_final
    elif (i == 1):
        contour_lst2 = contours_final
    else:
        contour_lst3 = contours_final
        
    #cv2.namedWindow("image_nor_filter", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("filter_thresh", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("erosion", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("detect_edges_filter", cv2.WINDOW_NORMAL)
    #cv2.namedWindow("image_final", cv2.WINDOW_NORMAL)
    #cv2.imshow("filter_thresh", filter_thresh)
    #cv2.imshow("image_nor_filter", image_nor_filter)
    #cv2.imshow("image_final", image_final)
    #cv2.imshow("detect_edges_filter", detect_edges_filter)
    #cv2.imshow("erosion", erosion)
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    contours_final = None
    image_final = None
  
'''
下一步：提取边缘，映射到实际尺寸上
'''
contour_lst1 = flatten(contour_lst1)
contour_lst2 = flatten(contour_lst2)
contour_lst3 = flatten(contour_lst3)
contour_lst = []
contour_lst.append(contour_lst1)
contour_lst.append(contour_lst2)
contour_lst.append(contour_lst3)
contour_num = 3

# 泛化：需要动态生成列表，类似生成线性表的方法
contour_lst1X = []
contour_lst1Y = []
contour_lst2X = []
contour_lst2Y = []
contour_lst3X = []
contour_lst3Y = []

contour_allX = []                       # Extracting by using the following form is recommended:
contour_allX.append(contour_lst1X)      # contour_allX[0]
contour_allX.append(contour_lst2X)      # contour_allX[1]
contour_allX.append(contour_lst3X)      # contour_allX[2]
contour_allY = []
contour_allY.append(contour_lst1Y)
contour_allY.append(contour_lst2Y)
contour_allY.append(contour_lst3Y)

move_up_1_X = []
move_up_1_Y = []
move_up_2_X = []
move_up_2_Y = []
move_across_1_X = []
move_across_1_Y = []
move_across_2_X = []
move_across_2_Y = []
move_down_1_X = []
move_down_1_Y = []
move_down_2_X = []
move_down_2_Y = []

count = 0
while count < contour_num:
    for i in range(len(contour_lst[count])):
        if (i % 2 == 0):
            contour_lst[count][i] = gray.shape[1] - contour_lst[count][i]
        else:
            contour_lst[count][i] = gray.shape[0] - contour_lst[count][i]
    count = count + 1
            
scalar = 0.05       # Size of each pixel
count = 0
i = None
while count < contour_num:
    for i in range(len(contour_lst[count])):
        contour_lst[count][i] = ((20 + scalar * contour_lst[count][i]) / 100)
    count = count + 1

count = 0
#even = 0
#odd = 0
i = None
while count < contour_num:
    #fx = open("contour_" + str(count) + "x_final.txt", 'w')
    #fy = open("contour_" + str(count) + "y_final.txt", 'w')
    for i in range(len(contour_lst[count])):
        if (i % 2 == 0):
            contour_allY[count].append(contour_lst[count][i])
            #print >> fy, "%.5f, " %((contour_lst[count][i]))
            #even = even + 1
        else:
            contour_allX[count].append(contour_lst[count][i])
            #print >> fx, "%.5f, " %(contour_lst[count][i])
            #odd = odd + 1
    #print "even = %d odd = %d" %(even, odd)
    #even = 0
    #odd = 0
    #fx.close()
    #fy.close()
    count = count + 1

'''
move the pen
'''
# move up
Z_up_1 = []
Z_up_2 = []
Z0 = 0.3
offset = 0.1

step = 50
i = 0
while i <= step:
    temp = contour_allX[0][len(contour_allX[0]) - 1]
    move_up_1_X.append(temp)
    temp = contour_allY[0][len(contour_allY[0]) - 1]
    move_up_1_Y.append(temp)
    Z_up_1.append(Z0 + offset * i / step)
    i = i + 1

i = 0
while i <= step:
    temp = contour_allX[1][len(contour_allX[0]) - 1]
    move_up_2_X.append(temp)
    temp = contour_allY[1][len(contour_allY[0]) - 1]
    move_up_2_Y.append(temp)
    Z_up_2.append(Z0 + offset * i / step)
    i = i + 1

# print "test = %f" %contour_allY[1][0]
# move across
step = 25
i = 0
while i<= step:
    temp = contour_allX[0][len(contour_allX[0]) - 1] + ((contour_allX[1][0] - contour_allX[0][len(contour_allX[0]) - 1]) * i / step)
    move_across_1_X.append(temp)
    temp = contour_allY[0][len(contour_allY[0]) - 1] + ((contour_allY[1][0] - contour_allY[0][len(contour_allY[0]) - 1]) * i / step)
    move_across_1_Y.append(temp)
    i = i + 1

i = 0
while i<= step:
    temp = contour_allX[1][len(contour_allX[1]) - 1] + ((contour_allX[2][0] - contour_allX[1][len(contour_allX[1]) - 1]) * i / step)
    move_across_2_X.append(temp)
    temp = contour_allY[1][len(contour_allY[1]) - 1] + ((contour_allY[2][0] - contour_allY[1][len(contour_allY[1]) - 1]) * i / step)
    move_across_2_Y.append(temp)
    i = i + 1

# move down
Z_down_1 = []
Z_down_2 = []
    
step = 50
i = 0
while i <= step:
    temp = contour_allX[1][0]
    move_down_1_X.append(temp)
    temp = contour_allY[1][0]
    move_down_1_Y.append(temp)
    Z_down_1.append(Z0 + offset * (1.0 - (float(i) / float(step))))
    #print "test = %f" %(Z0 + offset * (1.0 - (float(i) / float(step))))
    i = i + 1

i = 0
while i <= step:
    temp = contour_allX[2][0]
    move_down_2_X.append(temp)
    temp = contour_allY[2][0]
    move_down_2_Y.append(temp)
    Z_down_2.append(Z0 + offset * (1.0 - (float(i) / float(step))))
    #print "test = %f" %(Z0 + offset * (1.0 - (float(i) / float(step))))
    i = i + 1
    
'''
所有需要的列表：
Z_down_1
Z_down_2
move_up_1_X
move_up_1_Y
move_up_2_X
move_up_2_Y
move_across_1_X
move_across_1_Y
move_across_2_X
move_across_2_Y
move_down_1_X
move_down_1_Y
move_down_2_X
move_down_2_Y

备注：
第一次下笔速度控制也应当考虑，具体实现应在你求解端给出
具体需不需要考虑可以借鉴之前画A，因为当时我不在现场，并不知道效果如何
'''
##############################################################################
# Add the goal here
goal = TrajGoal()
goal.offset = [0.0, 0.0, 0.0]
goal.x = []
goal.y = []
goal.z = [0.3]*len(goal.x)
# Send the goal
client.send_goal(goal, feedback_cb=feedback_cb)
# Wait the result 
client.wait_for_result()