import time


class PID_Controller():
    def __init__(self):
        self.K_p = 0.1
        self.K_d = 0.1
        self.K_i = 0.1

        self.init_values()

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

    def get_PID(self, error):

        self.current_time = time.time()
        dt = self.current_time - self.previous_time
        de = error - self.prev_error

        self.influence_p = self.K_p * error  # Proportinal
        self.influence_i += error * dt  # Integral

        self.influence_p = 0
        if dt > 0:
            self.influence_d = de / dt  # Derrivitive

        self.previous_time = self.current_time
        self.prev_error = error

        # sum the terms and return the result
        return self.influence_p + (self.K_i * self.influence_i) + (self.K_d * self.influence_d)

