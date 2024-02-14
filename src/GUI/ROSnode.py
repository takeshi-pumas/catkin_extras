import rospy
from geometry_msgs.msg import Twist


class BASE_CONTROLLER:
    def __init__(self, topic):
        self._base_vel_pub = rospy.Publisher(
            '/hsrb/command_velocity', Twist, queue_size=1)
        self.MAX_VEL = 0.005

    def forward(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velX= vel)
    def backward(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velX = -vel)
    def right(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velY = -vel)
    def left(self, slider_value):
        vel = slider_value * self.MAX_VEL
        self._publish_msg(velY = vel)
    def turn_l(self, slider_value):
        vel = slider_value * 2 * self.MAX_VEL
        self._publish_msg(velT= -vel)
    def turn_r(self, slider_value):
        vel = slider_value * 2 * self.MAX_VEL
        self._publish_msg(velT = vel)
    def stop(self):
        self._publish_msg()


    def _publish_msg(self, velX = 0, velY = 0, velT= 0):
        twist = Twist()
        twist.linear.x = velX
        twist.linear.y = velY
        twist.angular.z = velT
        self._base_vel_pub.publish(twist)



