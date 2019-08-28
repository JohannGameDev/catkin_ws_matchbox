import time
import rospy
from autominy_msgs.msg import SpeedCommand


class Speed_Controller():
    def init(self):
        self.speed_sub = rospy.Subscriber("/sensors/speed", SpeedCommand, self.get_speed, queue_size=10)
        self.K_p = 0.1
        self.K_d = 0.1
        self.K_i = 1.0
        self.desired_speed = 0.3
        self.speed_value = 0
        self.init_values()

    def set_desired_speed(self, speed):
        self.desired_speed = speed

    def set_K_p(self, k_p):
        # proportional
        self.K_p = k_p

    def set_K_i(self, k_i):
        #integral
        self.K_i = k_i

    def set_K_d(self, k_d):
        # derrivitive
        self.K_d = k_d

    def set_last_error(self, prev_error):
        # last error
        self.prev_error = prev_error

    def init_values(self):
        # init time
        self.current_time = time.time()
        self.previous_time = self.current_time

        self.prev_error = 0

        # after calculation
        self.influence_p = 0
        self.influence_i = 0
        self.influence_d = 0

    def get_speed(self, msg):
        current_speed = msg.value
        error = self.desired_speed - current_speed
        #if abs(error) < 0.2:
        #self.speed_value = current_speed
        #else:
        self.current_time = time.time()
        dt = self.current_time - self.previous_time
        de = error - self.prev_error

        self.influence_p = self.K_p * error  # Proportinal
        self.influence_i += error * dt  # Integral

        if dt > 0:
            self.influence_d = de / dt  # Derrivitive

        self.previous_time = self.current_time
        self.prev_error = error

        # sum the terms
        self.speed_value = self.influence_p + (self.K_i * self.influence_i) + (self.K_d * self.influence_d)
        #self.speed_value = 0.3