import numpy as np
import rospy
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point,PointStamped




class SplineInterpolation():
    def __init__(self):
        self.lane1 = np.load('/home/johann/catkin_ws_matchbox/src/assignment8_splines/data/lane1.npy')
        self.lane2 = np.load('/home/johann/catkin_ws_matchbox/src/assignment8_splines/data/lane2.npy')
        self.lane1 = self.lane1[[0, 100, 150, 209, 259, 309, 350, 409, 509, 639, 750, 848, 948, 1028, 1148, 1276], :]
        self.lane2 = self.lane2[[0, 50, 100, 150, 209, 400, 600, 738, 800, 850, 900, 949, 1150, 1300, 1476,], :]
        self.line_publisher_1 = rospy.Publisher("/interpolation/line_strip_1", Marker, queue_size=100)
        self.line_publisher_2 = rospy.Publisher("/interpolation/line_strip_2", Marker, queue_size=100)
        self.clicked_publisher = rospy.Publisher("/interpolation/clicked_point", Marker, queue_size=100)
        self.closest = rospy.Publisher("/interpolation/closest_point", Marker, queue_size=100)


        self.xy_1 = 0
        self_xy_2 = 0

        rospy.Subscriber("/clicked_point",PointStamped,self.process_clicked_point)

    def process_clicked_point(self,pointst):
        print(pointst.point.x)
        self.publish_clicked(pointst)
        closest_point = self.closest_point((pointst.point.x,pointst.point.y),self.xy_1)
        print(closest_point)
        self.publish_closest(closest_point)

    def publish_clicked(self,pointst):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = pointst.point.x
        marker.pose.position.y = pointst.point.y
        marker.pose.position.z = 0.0

        # marker.points = []
        #
        # clicked_point = Point()
        # clicked_point.x = pointst.point.x
        # clicked_point.y = pointst.point.y
        # clicked_point.z = 0.0
        # marker.points.append(clicked_point)
        self.clicked_publisher.publish(marker)

    def publish_closest(self,closest):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.SPHERE
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # marker color
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = closest[0]
        marker.pose.position.y = closest[1]
        marker.pose.position.z = 0.0

        # marker.points = []
        #
        # clicked_point = Point()
        # clicked_point.x = closest[0]
        # clicked_point.y = closest[1]
        # clicked_point.z = 0.0
        # marker.points.append(clicked_point)
        #
        self.closest.publish(marker)

    def getArc(self):
        # lane 1
        arc_lane1 = self.lane1[:,0]
        X_lane1 = self.lane1[:,1]
        Y_lane1 = self.lane1[:,2]

        splineX_lane1 = CubicSpline(arc_lane1, X_lane1)
        splineY_lane1 = CubicSpline(arc_lane1, Y_lane1)

        # LANE2
        arc_lane2 = self.lane2[:, 0]
        X_lane2 = self.lane2[:, 1]
        Y_lane2 = self.lane2[:, 2]

        splineX_lane2 = CubicSpline(arc_lane2, X_lane2)
        splineY_lane2 = CubicSpline(arc_lane2, Y_lane2)

        xs = np.arange(0,13,0.1)

        xy_lane1 = list(zip(splineX_lane1(xs),splineY_lane1(xs)))
        xy_lane2 = list(zip(splineX_lane2(xs),splineY_lane2(xs)))

        self.xy_1 = xy_lane1
        self.xy_2 = xy_lane2

        return xy_lane1,xy_lane2


    def create_marker(self,xy_lane):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points = []
        id = 0
        for xy in xy_lane:
                line_point = Point()
                line_point.x = xy[0]
                line_point.y = xy[1]
                line_point.z = 0.0
                marker.points.append(line_point)

        return marker
    def create_simple_marker(self):
        marker = Marker()
        marker.header.frame_id = "/map"
        marker.type = marker.LINE_STRIP
        marker.action = marker.ADD

        # marker scale
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        # marker color
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        # marker orientaiton
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # marker position
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0

        marker.points = []

        line_point = Point()
        line_point.x = 1
        line_point.y = 1
        line_point.z = 0.0
        marker.points.append(line_point)

        line_point2 = Point()
        line_point2.x = 2
        line_point2.y = 2
        line_point2.z = 0.0
        marker.points.append(line_point2)

        return marker

    def distance_2_points(self,p1, p2):
        p1 = np.array((p1[0], p1[1]))
        p2 = np.array((p2[0], p2[1]))

        return np.linalg.norm(p1 - p2)

    def closest_point(self,position, spline):
        point = []
        distance = 999999
        for p in spline:
            if self.distance_2_points(position, p) <= distance:
                distance = self.distance_2_points(position, p)
                point = p
        return point




if __name__ == '__main__':
    rospy.init_node('line_interpol', anonymous=True)
    spline_interpolation = SplineInterpolation()
    xy_1,xy_2=spline_interpolation.getArc()
    marker_1 = spline_interpolation.create_marker(xy_1)
    marker_2 = spline_interpolation.create_marker(xy_2)

    while not rospy.is_shutdown():
        spline_interpolation.line_publisher_1.publish(marker_1)
        spline_interpolation.line_publisher_2.publish(marker_2)

        rospy.sleep(0.01)

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()

    except KeyboardInterrupt:
        print("Shutting down")

