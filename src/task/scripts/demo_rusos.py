from restaurant_utils import *


#point_publisher = rospy.Publisher("/clicked_point", PointStamped)

if __name__ == "__main__":
    while not rospy.is_shutdown:
        #punto = PointStamped()
        x = 2 * np.random.random()
        y = 2 * np.random.random()
        succ = omni_base.move_base(goal_x = x, goal_y = y)
