from map import Lane,Map
from pid import PID
import rospy
from autominy_msgs.msg import SpeedCommand, NormalizedSteeringCommand, SteeringCommand
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
import math
import numpy as np
import tf.transformations
from visualization_msgs.msg import Marker






class Navigation():

    def __init__(self):
        rospy.init_node("Navigation")

        self.map = Map()
        self.pid = PID()
        self.lane = 0
        self.lane_pub = rospy.Publisher('lane_num', Int32, queue_size=10)
        self.lane_pub.publish(self.lane)
        self.lane_sub = rospy.Subscriber("/lane_num", Int32, self.set_lane, queue_size=10)
        self.localization_sub = rospy.Subscriber("/sensors/localization/filtered_map", Odometry, self.on_localization,
                                                 queue_size=10)
        self.pid_desired_angle = rospy.Publisher("/desired_angle/angle", NormalizedSteeringCommand, queue_size=10)
        self.lookahead_pub = rospy.Publisher("lane", Marker, queue_size=10)

        self.rate = rospy.Rate(100)
        rospy.on_shutdown(self.on_shutdown)
        while not rospy.is_shutdown():
            self.rate.sleep()

    def set_lane(self, lane_num):
        self.lane = lane_num.data

    def on_localization(self,data):
        current_pos = np.array([data.pose.pose.position.x,data.pose.pose.position.y])
        look_ahead_point = self.map.lanes[self.lane].lookahead_point(current_pos,0.5)[0]# [0] for lookahead point  inner lane
        self.publish_looakhead(look_ahead_point)
        car_yaw = self.get_car_yaw(data)
        yaw = self.calculateYaw(current_pos,look_ahead_point,car_yaw) # yaw difference from two points
        print("I want this yaw:"+str(yaw))
        steering_msg = NormalizedSteeringCommand()
        steering_msg.value = yaw
        self.pid_desired_angle.publish(steering_msg)

    def calculateYaw(self,current_pos,look_ahead_point,car_yaw):

        v = [look_ahead_point[0] - current_pos[0], look_ahead_point[1] - current_pos[1]]
        theta = car_yaw
        c, s = np.cos(theta), np.sin(theta)
        R = np.array(((c, -s), (s, c)))
        v_r = np.matmul(v, R)
        myradians = math.atan2(v_r[1], v_r[0])
        return myradians


    def get_car_yaw(self,data):
        quat = [data.pose.pose.orientation.x, data.pose.pose.orientation.y, data.pose.pose.orientation.z,
                data.pose.pose.orientation.w]
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quat)
        return yaw

    def on_shutdown(self):
        speed_msg = SpeedCommand()
        speed_msg.value = 0.0
        self.pid.speed_pub.publish(speed_msg)

    def publish_looakhead(self,look_ahead):
        i= 0
        msg = Marker(type=Marker.SPHERE, action=Marker.ADD)
        msg.header.frame_id = "map"
        msg.scale.x = 0.2
        msg.scale.y = 0.2
        msg.scale.z = 0.2
        msg.color.b = 1.0
        msg.color.a = 1.0
        msg.id = i

        #p, param = lane.closest_point(point)
        msg.pose.position.x = look_ahead[0]
        msg.pose.position.y = look_ahead[1]

        # i += 1
        #
        # self.lane_pub.publish(msg)
        # # green lookahead point
        # msg.color.b = 0.0
        # msg.color.g = 1.0
        # p, param = lane.lookahead_point(point, 0.5)
        # msg.pose.position.x = p[0]
        # msg.pose.position.y = p[1]
        # msg.id = i
        #
        # i += 1

        self.lookahead_pub.publish(msg)

if __name__ == "__main__":
    Navigation()