#! /usr/bin/env python
import rospy
import time
import actionlib
import cv2
import numpy as np
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

# Use resize instaed
def im_res(im):
    # Keep the aspect ration of the image
    # Limit the workspace for drawing
    # The max workspace is 600mm x 600mm
    print("The image shape before resize: ",im.shape)
    x = im.shape[0]
    y = im.shape[1]
    y_max = 450.0
    x_max = 450.0
    fy = y_max/y
    fx = x_max/x
    
    if(y >= x):
        im_resize = cv2.resize(im, None, fx=fy, fy=fy, interpolation=cv2.INTER_AREA)
    else:
        im_resize = cv2.resize(im, None, fx=fx, fy=fx, interpolation=cv2.INTER_AREA)
    print('The image shape after resize: ' , im_resize.shape)
    return im_resize
##############################################################################
# Test the basic Opencv operation
# Please use absolute path, otherwise it will cause error
im = cv2.imread('/home/tlluo36/catkin_ws/src/generate_traj/data/opencv-logo.png')
# Resize the image to fit in workspace
im = im_res(im)
cv2.imshow('src_im', im)
imgray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
# Find contours
ret, thresh = cv2.threshold(imgray, 244, 255, 0)
contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
print(len(contours))

# The contours to be print
cnt_number = 1
print('\n''cnt_number = %d' %(cnt_number))
# Filter the contours and print info
contours2=[cnt for cnt in contours] 
length_of_contours2 = len(contours2[cnt_number])
print('We already found the contours, let me show you more details......')
print("The length of contours2[%d]: %d" %(cnt_number,length_of_contours2))
print('The shape  this contours2[%d]: ' %(cnt_number))
print(np.array(contours2[cnt_number]).shape)
# Conversion: 1m = 1000mm
# Please make sure use float type(pay attention to 1000.0 below)
x_y_list = [round(c/(1000.0), 3) for c in contours2[cnt_number].flatten()]
# Reshape the x_y list
x_y_array = np.array(x_y_list).reshape(length_of_contours2,2)

# Del the repeat element in the array
def delrepeat(source):
    # Input and Ouput are both array
    # Del the repeat element in the array
    unique=[]
    for i in source.tolist():
        if i not in unique:
            unique.append(i)
    return np.array(unique)
# Del the repeat element in the array
x_y_array_unique = delrepeat(x_y_array)

# Get the x,y from the list
x_waypoints = (x_y_array_unique.flatten(1)[:length_of_contours2])
y_waypoints = (x_y_array_unique.flatten(1)[length_of_contours2:])
# Output the result
print(len(x_y_array_unique))
print(x_waypoints)

# Add the goal here
goal = TrajGoal()
goal.offset = [0.0, 0.0, 0.0]
# goal.x = x_waypoints
# goal.y = y_waypoints
goal.x = []
goal.y = []
# goal.z = [0.3]*length_of_contours2
goal.z = [0.3]*len(goal.x)
# Send the goal
client.send_goal(goal, feedback_cb=feedback_cb)

## Uncomment these lines to test goal preemption:
#time.sleep(3.0)
#client.cancel_goal()

# Wait the result 
client.wait_for_result()
print('[Result] State: %d'%(client.get_state()))
print('[Result] Status: %s'%(client.get_goal_status_text()))
# print('[Result] Time elapsed: %f'%(client.get_result().time_elapsed.to_sec()))
# print('[Result] Updates sent: %d'%(client.get_result().updates_sent))
