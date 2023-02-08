#pylint: disable-all
import cv2 as cv
import numpy as np
import pyzed.sl as sl
import math
import rospy
import tf
from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped

fx=1066.17
fy=1066.72
cx=953.88
cy=532.349
k1=-0.0476521
k2=0.0212008
p1=-4.48125e-05
p2=-0.000737505
k3=-0.00895334

Z_OFFSET_CAMERA=1.19
Y_OFFSET_CAMERA=0.94
X_OFFSET_CAMERA=0

def rad_to_deg(th):
    return th*(180/math.pi)

def draw(img, corners, imgpts):
    c1 = int(corners[0].ravel()[0])
    c2 = int(corners[0].ravel()[1])
    corner = [c1, c2]

    # print(corner)

    v1 = int(imgpts[0].ravel()[0])
    v2 = int(imgpts[0].ravel()[1])
    # print([v1, v2])
    img = cv.line(img, corner, [v1,v2], (255,0,0), 5)

    v1 = int(imgpts[1].ravel()[0])
    v2 = int(imgpts[1].ravel()[1])
    img = cv.line(img, corner, [v1,v2], (0,255,0), 5)

    v1 = int(imgpts[2].ravel()[0])
    v2 = int(imgpts[2].ravel()[1])
    img = cv.line(img, corner, [v1,v2], (0,0,255), 5)

    return img



def main():
    pub = rospy.Publisher('/cube_pose', PoseStamped, queue_size=10)
    rospy.init_node('cube_pose_publisher')

    # Create a ZED camera object
    zed = sl.Camera()

    # Set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD720
    init_params.camera_fps = 30
    init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
    init_params.coordinate_units = sl.UNIT.METER
    init_params.depth_minimum_distance = 0.5
    init_params.depth_maximum_distance = 1.5

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("error")
        exit(-1)
    image = sl.Mat()
    point_cloud = sl.Mat()
    runtime_parameters = sl.RuntimeParameters()
    runtime_parameters.sensing_mode = sl.SENSING_MODE.FILL
    runtime_parameters.confidence_threshold = 100
    runtime_parameters.textureness_confidence_threshold = 100
    i=1
    while not rospy.is_shutdown():
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # A new image is available if grab() returns SUCCESS
            zed.retrieve_image(image, sl.VIEW.LEFT) # Retrieve the left image
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            img = image.get_data()
            gray = cv.cvtColor(img,cv.COLOR_RGB2GRAY)
            
            row = 3
            col = 4
            ret, corners = cv.findChessboardCorners(gray, (row,col), cv.CALIB_CB_FAST_CHECK)
            objp = np.zeros((row*col,3), np.float32)
            objp[:,:2] = np.mgrid[0:row,0:col].T.reshape(-1,2)
            axis = np.float32([[3,0,0], [0,3,0], [0,0,-3]]).reshape(-1,3)
            mtx = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
            dist = np.empty(4)
            if ret == True:
                # Find the rotation and translation vectors.
                ret, rvecs, tvecs = cv.solvePnP(objp, corners, mtx, dist)
                # rMat, _ = cv.Rodrigues(rvecs)
                print("----------------------------------")
                c1 = int(corners[0].ravel()[0])
                c2 = int(corners[0].ravel()[1])
                print(c1, c2)
                err, xyz = point_cloud.get_value(c1,c2)
                print("xyz: "+str(xyz))
                print("==================================")
                imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
                # print(imgpts)
                if imgpts is None:
                    
                    # print("===================")
                    continue
                else:
                    img = draw(img, corners, imgpts)
                    cv.imshow("CHess", img)
                    #cv.imwrite('/output/out.png',img)

                    cube_pos = PoseStamped()
                    # x axis is -x axis of camera
                    cube_pos.pose.position.x = -Y_OFFSET_CAMERA + xyz[2]
                    # y axis is z axis of camera
                    cube_pos.pose.position.y = -X_OFFSET_CAMERA+xyz[0]
                    # z axis is -y axis of camera
                    cube_pos.pose.position.z = Z_OFFSET_CAMERA - xyz[1]
                    
                    cube_pos.header =  Header()
                    cube_pos.header.frame_id = "map"
                    quaternion = tf.transformations.quaternion_from_euler(rvecs[2],rvecs[1],-(rvecs[0]))
                    cube_pos.pose.orientation.x = quaternion[0]
                    cube_pos.pose.orientation.y = quaternion[1]
                    cube_pos.pose.orientation.z = quaternion[2]
                    cube_pos.pose.orientation.w = quaternion[3]

                    pub.publish(cube_pos)

                    cv.waitKey(1)
                    i+=1
            print("iteration")

if __name__ == "__main__":
    main()

