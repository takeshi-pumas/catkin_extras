#!/usr/bin/env python3

#import rospy

# import actionlib
# from hmm_navigation.msg import NavigateActionGoal, NavigateAction

# navclient = actionlib.SimpleActionClient('/navigate_hmm', NavigateAction)   ### HMM NAV

def move_base(goal_x,goal_y,goal_yaw,time_out=10):
    nav_goal = NavigateActionGoal()
    nav_goal.goal.x = goal_x
    nav_goal.goal.y = goal_y
    nav_goal.goal.yaw = goal_yaw
    nav_goal.goal.timeout = time_out

    print (nav_goal)

    # send message to the action server
    navclient.send_goal(nav_goal.goal)

    # wait for the action server to complete the order
    navclient.wait_for_result(timeout=rospy.Duration(time_out))

    # print result of navigation
    action_state = navclient.get_state()
    return navclient.get_state()

def request():
	trans = [0,0,0]
	rot = [0,0,0,1]
	print("Where do you want to go?")
	goal = input()
	goal = goal.casefold()

	#Read the "known location" file 
	with open('test.txt','r') as known_loc:
		lines = known_loc.readlines()
    #Match goal with known locations 
	for line in lines[1:]:
		line = line.replace('\n', '')
		line = line.replace(' ', '')
		name,_ = line.split(',',1)
		succ = name.casefold() == goal
		if succ:
			name,trans[0],trans[1],trans[2],_ = line.split(',',4)
			for i,t in enumerate(trans):
				trans[i] = int(t)
			break
	if not succ:
		print("Location goal is not valid")
		# return succ, [0,0,0]
	# else:
	return succ, trans

def action():
	#Ros node start
	rospy.init_node('goto', anonymous=True)

	# action
	succ, trans = request()
	if succ:
		return move_base(*trans,time_out = 20)

	#to keep node alive 
	rospy.spin()
	
if __name__ == "__main__":
	action()