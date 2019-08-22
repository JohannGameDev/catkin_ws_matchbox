
# --- imports ---
import rospy
from math import sqrt
from std_msgs.msg import UInt8
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
from PID_Controller import PID_Controller
from tf.transformations import euler_from_quaternion, quaternion_from_euler



class Drive_PID():
    def __init__(self,desired_angle):

        # #
        self.desired_angle = desired_angle
        self.speed_cmd = SpeedCommand()


        self.speed_cmd.value = 0.3
        self.PID_controller = PID_Controller()
        self.pub_steering = rospy.Publisher("/actuators/steering",SteeringCommand,queue_size=100)
        self.pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=100)


        pub_speed = rospy.Publisher("/actuators/speed", SpeedCommand, queue_size=10)

        # angle_left = 30 angle_straight = 90 angle_right = 150

        gps_subscriber = rospy.Subscriber("/communication/gps/3", Odometry, self.callbackGPS) # subscribe to topic "/sensor/speed
        #self.pub_speed.publish(self.speed_cmd)



    def callbackGPS(self,gps_data):

        # calculate how off steering is and give the error to PID Controler
        orientation = gps_data.pose.pose.orientation
        #print(gps_data)
        current_yaw =  self.get_rotation(orientation)
        error =  self.desired_angle - current_yaw
        update = self.PID_controller.get_PID(error)
        print(current_yaw)
        print(update)
        steering = SteeringCommand()
        steering.value = update
        self.pub_steering.publish(steering)

    def get_rotation(self,q):
        orientation_q = q
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

if __name__ == '__main__':
    rospy.init_node('PID', anonymous=True)
    pub_speed = rospy.Publisher('/actuators/speed', SpeedCommand, queue_size=10)

    # speed init
    speed_cmd = SpeedCommand()
    speed_cmd.value = 0.1
    rate = rospy.Rate(5)  # 5 hz
    pub_speed.publish(speed_cmd)
    drive_pid = Drive_PID(90)

    while not rospy.is_shutdown():
         pub_speed.publish(speed_cmd)
         rate.sleep()


    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()

    except KeyboardInterrupt:
        speed_cmd = SpeedCommand()
        speed_cmd.value = 0
        pub_speed.publish(speed_cmd)

        print("Shutting down")