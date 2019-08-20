import rospy
from cv_bridge import CvBridge, CvBridgeError
import cv2
from sensor_msgs.msg import Image,CameraInfo
import numpy as np
from ransac import Ransac



class Draw_Lanes():

    def __init__(self):
        self.image_pub_bw = rospy.Publisher("/image_processing/grey_to_bw", Image, queue_size=1)
        self.image_pub_lines= rospy.Publisher("/image_processing/lines", Image, queue_size=1)# line publisher
        self.ransac = Ransac()

        infra_cam_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.callback_cam_image)

        self.bridge = CvBridge()

    def callback_cam_image(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            bw_image = self.gray_to_black_white(cv_image)
            self.image_pub_bw.publish(self.bridge.cv2_to_imgmsg(bw_image, "mono8"))

            # left line
            x_left, y_left = self.ransac.getCoordinatesBwImage(bw_image,"firsthalf")
            x_y_left = list(zip(x_left, y_left))
            m_b_left = self.ransac.perform_ransac(x_y_left, 4, 100)
            calc_point = m_b_left[0] * 640 + m_b_left[1]
            cv2.line(cv_image, (0, m_b_left[1]), (640, calc_point), 128, 5)
            print("Left line: Slope: {} , b: {}".format(m_b_left[0],m_b_left[1]))
            self.ransac.clear()
            # right line
            x_right, y_right = self.ransac.getCoordinatesBwImage(bw_image, "secondhalf")
            x_y_right = list(zip(x_right, y_right))
            m_b_right = self.ransac.perform_ransac(x_y_right, 4, 100)
            calc_point = m_b_right[0] * 640 + m_b_right[1]
            cv2.line(cv_image, (0, m_b_right[1]), (640, calc_point), 128, 5)
            print("right line: Slope: {} , b: {}".format(m_b_right[0],m_b_right[1]))




            self.image_pub_lines.publish(self.bridge.cv2_to_imgmsg(cv_image, "mono8"))

        except CvBridgeError as e:
            print(e)

            #self.image_pub_cluster.publish(self.bridge.cv2_to_imgmsg(bw_image, "mono8"))

    def gray_to_black_white(self,image):
            # bi_gray
            bi_gray_max = 255
            bi_gray_min = 242
            ret, thresh = cv2.threshold(image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)

            ## black rectangles over the remaining white pixel
            cv2.rectangle(thresh, (0, 248), (640, 480), 0, -1) # bottom
            cv2.rectangle(thresh, (0, 0), (640, 90), 0, -1) # top
            # triangle as contour, to get the left to top line and qr code away
            pt1 = (0, 0) # top left corner
            pt2 = (0, 345) # left bottom
            pt3 = (240, 0) # top right
            triangle_cnt = np.array([pt1, pt2, pt3])
            cv2.drawContours(thresh, [triangle_cnt], 0, 0, -1)


            return thresh

if __name__ == '__main__':
    rospy.init_node('draw_lanes', anonymous=True)

    draw_Lanes = Draw_Lanes()

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")