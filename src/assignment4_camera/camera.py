import rospy
from sensor_msgs.msg import Image,CameraInfo
from sklearn.cluster import KMeans
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
from numpy.linalg import inv
import math



class FindMarker():
    def __init__(self):
        self.image_pub_bw = rospy.Publisher("/image_processing/grey_to_bw", Image, queue_size=1)
        self.image_pub_cluster = rospy.Publisher("/image_processing/cluster_center", Image, queue_size=1)

        cam_para = rospy.Subscriber("/sensors/camera/infra1/camera_info", CameraInfo, self.callback_cam_para)

        infra_cam_sub = rospy.Subscriber("/sensors/camera/infra1/image_rect_raw", Image, self.callback_cam_image)
        self.bridge = CvBridge()



    def callback_cam_para(self,data):
        print(data)



    def callback_cam_image(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data)
            bw_image = self.gray_to_black_white(cv_image)
            self.image_pub_bw.publish(self.bridge.cv2_to_imgmsg(bw_image, "mono8"))
            marker_positions = self.find_white_pixel(bw_image)
           # print(marker_positions) # a array of arrays, every array contains y x coordinate of cluster
            for pos in marker_positions:
                cv2.circle(bw_image, (int(pos[1]),int(pos[0])), 10, 255, -1)

            self.image_pub_cluster.publish(self.bridge.cv2_to_imgmsg(bw_image, "mono8"))

            #cv2.imwrite('/home/johann/black_white.bmp', bw_image)
            retval,rvec,tvec = self.solve_pnp(marker_positions)

            print("Solve PNP")
            print("rvec")
            print(rvec)
            print("tvec")
            print(tvec)
            print("Rotation")
            rot, jac = self.rod(rvec)
            print(rot)
            A = np.vstack((rot, [0, 0, 0]))
            B = np.vstack((tvec, [1]))
            homo = np.hstack((A, B))
            print("homogenous")
            print(homo)

            ainv = inv(homo)
            print("inverse")
            print(ainv)
            print("euler,pitch,yaw,roll")
            # pitch up + down -
            # yaw left or right face to face + to -
            # roll roll face left to right + to -
            euler = self.rotationMatrixToEulerAngles(rot)
            print(euler)

        except CvBridgeError as e:
            print(e)

    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    def rotationMatrixToEulerAngles(self,R):
        assert (self.isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def rod(self,rvec):
        rmat = np.zeros((3, 3))
        dst, jac = cv2.Rodrigues(rvec, rmat, jacobian=0)
        return dst, jac

    def gray_to_black_white(self,image):
            # bi_gray
            bi_gray_max = 255
            bi_gray_min = 242
            ret, thresh = cv2.threshold(image, bi_gray_min, bi_gray_max, cv2.THRESH_BINARY)

            ## black rectangles over the remaining white pixel
            cv2.rectangle(thresh, (0, 248), (640, 480), 0, -1) # bottom
            cv2.rectangle(thresh, (0, 0), (640, 90), 0, -1) # top


            return thresh


    def find_white_pixel(self,image):
        all_positions = []

        for y in range(image.shape[0]):
            for x in range(image.shape[1]):
                if image[y, x] == 255:
                    all_positions.append([y, x])

        X = np.array(all_positions)
        kmeans = KMeans(n_clusters=6, random_state=0).fit(X)

        return kmeans.cluster_centers_

    def solve_pnp(self,img_points):
        fx = 383.7944641113281
        fy = 383.7944641113281
        cx = 322.3056945800781
        cy = 241.67051696777344
        camera_mat = np.zeros((3, 3, 1))
        camera_mat[:, :, 0] = np.array([[fx, 0, cx],
                                        [0, fy, cy],
                                        [0, 0, 1]])

        dist_coeffs = np.zeros((5, 1))
        # close to far, left to right (order of discovery) in cm
        obj_points = np.zeros((6, 3, 1))
        obj_points[:, :, 0] = np.array([[0.5, 0.3, 0],
                                        [0.5, -0.2, 0],
                                        [0.8, 0.2, 0],
                                        [0.8, -0.2, 0],
                                        [1.1, 0.2, 0],
                                        [00.0, -0.2, 0],
                                        ])
        retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, camera_mat, dist_coeffs)

        return retval, rvec, tvec

    # Checks if a matrix is a valid rotation matrix.
    def isRotationMatrix(self,R):
        # https://www.learnopencv.com/rotation-matrix-to-euler-angles/
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

if __name__ == '__main__':
    rospy.init_node('find_marker', anonymous=True)

    findMarker = FindMarker()

    # spin() simply keeps python from exiting until this node is stopped
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
#    cv2.destroyAllWindows()
