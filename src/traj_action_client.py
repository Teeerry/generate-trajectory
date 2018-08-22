#! /usr/bin/env python
import rospy
import time
import actionlib
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

# Add the goal here
goal = TrajGoal()
del goal.x [:]
goal.x.append(0.5)
goal.x.append(0.5)
goal.x.append(0.5)

del goal.y [:]
goal.y.append(0.3)
goal.y.append(0.4)
goal.y.append(0.5)

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