#Steps to perform ransac
#Sample random points min 2, probality for more points to be a good representation shrinks
# from this two points build a model (line)
# for all points you have calculate distance to that point
# count how many points are in a distance + sigma to - sigma, save it
# repeat it many iterations
# in the end take the points with the mosted counted in the area
import random as rnd
import matplotlib.pyplot as plt
import numpy as np
import cv2


class Ransac():
    def __init__(self):
        self.points_inline_count = {}


    def sample_points(self,xy_list):
        same = True
        while(same):
            rnd_first = xy_list[rnd.randint(0,len(xy_list)-1)]
            rnd_second = xy_list[rnd.randint(0,len(xy_list)-1)]
            if not rnd_first[1] == rnd_second[1]: # if y coordinates same will be divison by zero at slope
                same = False

        return rnd_first,rnd_second

    def get_line(self,rnd_first,rnd_second):
        m = (rnd_first[0] - rnd_second[0]) / (rnd_first[1] - rnd_second[1]) # m = y_1 - y_0 / x_1 - x_0
        b = rnd_first[1] - m * rnd_first[0] # y = mx +b <-> y - mx = b
        return m,b

    def get_distance(self,point,m,b):
        # distance will be f(x) value of m b, then distance to y
        f_x = m * point[0] + b
        dist_to_line = f_x - point[1]
        return dist_to_line

    def perform_ransac(self,xy_list,threshold,iterations):
        for i in range(0,iterations):
            first,second = self.sample_points(xy_list)
            m,b = self.get_line(first,second)
            inliner_count = 0
            for point in xy_list:
                distance = self.get_distance(point,m,b)
                abs_distance = self.absolute(distance)
                if abs_distance <= threshold:
                    inliner_count +=1
            self.points_inline_count[inliner_count] = (m,b)

        return self.get_best_fitting()

    def get_best_fitting(self):
        unsorted = list(self.points_inline_count.keys())
        best = max(unsorted)
        return self.points_inline_count[best]  # returns (m,b)

    def absolute(selfs,value):
        if value < 0:
            value *= -1
        return value

    # def getCoordinatesBwImage(self):
    #     img = cv2.imread('/home/johann/catkin_ws_matchbox/src/assignment6_ransac/bw.png', 0)
    #     xLen = img.shape[1]  # x-axes
    #     yLen = img.shape[0]  # y-axes
    #     self.xLen = xLen
    #     self.yLen = yLen
    #     X = []
    #     Y = []
    #     for y in range(int(yLen / 2), yLen):
    #         for x in range(0, xLen):
    #             if img[y, x] == 255:
    #                 X.append(x)
    #                 Y.append(y)
    #     return X,Y

    def getCoordinatesBwImage(self,img, param):
        if param == "firsthalf":
            lower_limit = 0
            upper_limit = img.shape[1] / 2  # x-axes
        elif param == "secondhalf":
            lower_limit = img.shape[1] / 2 + 1
            upper_limit = img.shape[1]  # x-axes second half
        print(img.shape[0])
        print(param)
        print(upper_limit)
        print(lower_limit)
        X = []
        Y = []

        for y in range(img.shape[0]):
            for x in range(lower_limit, upper_limit):
                if img[y, x] == 255:
                    X.append(x)
                    Y.append(y)

        return X,Y

    def clear(self):
        self.points_inline_count = {}

def main():
    ransac = Ransac()
    #x = [2,4,5,3,12,4,7,8,10]
    #y = [5,6,2,7,19,9,4,3,14]
    #x = np.random.randint(1000, size=1000)
    #y = np.random.randint(1000, size=1000)
    x,y = ransac.getCoordinatesBwImage()

    plt.plot(x,y,"bo")
    x_y = list(zip(x,y))
    m_b = ransac.perform_ransac(x_y,4,1000)

    g = np.linspace(0, 300, 100)
    h =  m_b[0] * g + m_b[1]
    plt.plot(g, h, '-r', label='y=2x+1')


    plt.show()

if __name__ == '__main__':
    main()








