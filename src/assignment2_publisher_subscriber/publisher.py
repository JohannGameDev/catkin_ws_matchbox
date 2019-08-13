#!/usr/bin/env python
import rospy
from autominy_msgs.msg import NormalizedSteeringCommand, SpeedCommand
#from std_msgs.msg import String



def talker():
    # Publish to this existing topics
    pub_steering = rospy.Publisher('/actuators/steering_normalized', NormalizedSteeringCommand, queue_size=10)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    rospy.init_node('steering_speed_whisperer', anonymous=True)

    # steering
    norm_steerin_cmd = NormalizedSteeringCommand()  # Command is a class create object
    norm_steerin_cmd.value = 1.0


    #speed init
    speed_cmd = SpeedCommand()
    speed_cmd.value = 0.3
    rate = rospy.Rate(5) # 5 hz
    while not rospy.is_shutdown():
        info_str = "time {time} : speed: {speed} , steering: {steering}".format(time=rospy.get_time(),speed=speed_cmd.value,steering=norm_steerin_cmd.value)
        rospy.loginfo(info_str)
        pub_steering.publish(norm_steerin_cmd)
        pub_speed.publish(speed_cmd)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass