import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import time
import threading
import numpy as np
import cv2.aruco as aruco
import sys, time, math


class Camera:

    def __init__(self):
        self.bridge = CvBridge()

        self.camera_image=rospy.Subscriber("/gi/simulation/left/image_raw", Image, self.image_callback)
        self.id_to_find  = 0
        self.marker_size  = 10 #- [cm]

        self.SHOW_FRAME  = True

        
        #--- Get the camera calibration path
        # calib_path  = ("/Users/vietdo/Documents/Dev/MarkerDetection")
        self.camera_matrix   = np.loadtxt("cameraMatrix.txt", delimiter=',')
        self.camera_distortion   = np.loadtxt("cameraDistortion.txt", delimiter=',')

        #--- 180 deg rotation matrix around the x axis
        self.R_flip  = np.zeros((3,3), dtype=np.float32)
        self.R_flip[0,0] = 1.0
        self.R_flip[1,1] =-1.0
        self.R_flip[2,2] =-1.0

        #--- Define the aruco dictionary
        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters  = aruco.DetectorParameters_create()

        #-- Font for the text in the image
        self.font = cv2.FONT_HERSHEY_PLAIN

        self.t_read      = time.time()
        self.t_detect    = self.t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0

    def update_fps_read(self):
        t           = time.time()
        self.fps_read    = 1.0/(t - self.t_read)
        self.t_read      = t
        
    def update_fps_detect(self):
        t           = time.time()
        self.fps_detect  = 1.0/(t - self.t_detect)
        self.t_detect      = t    


    def isRotationMatrix(self,R):
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype=R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

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




    def image_callback(self,msg):

        #print("Received an image!")
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            #cv2.imshow('image', cv2_img)        
            self.update_fps_read()
            
            #-- Convert in gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                    cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
            print(ids)
            if np.any(ids == self.id_to_find):

                self.update_fps_detect()
                #-- ret = [rvec, tvec, ?]
                #-- array of rotation and position of each marker in camera frame
                #-- rvec = [[rvec_1], [rvec_2], ...]    attitude of the marker respect to camera frame
                #-- tvec = [[tvec_1], [tvec_2], ...]    position of the marker in camera frame
                ret = aruco.estimatePoseSingleMarkers(corners, self.marker_size, self.camera_matrix, self.camera_distortion)

                #-- Unpack the output, get only the first
                rvec, tvec = ret[0][0,0,:], ret[1][0,0,:]

                #-- Draw the detected marker and put a reference frame over it
                aruco.drawDetectedMarkers(frame, corners)
                aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 10)

                #-- Obtain the rotation matrix tag->camera
                R_ct    = np.matrix(cv2.Rodrigues(rvec)[0])
                R_tc    = R_ct.T

                #-- Get the attitude in terms of euler 321 (Needs to be flipped first)
                roll_marker, pitch_marker, yaw_marker = self.rotationMatrixToEulerAngles(self.R_flip*R_tc)

                #-- Now get Position and attitude f the camera respect to the marker
                self.pos_camera = -R_tc*np.matrix(tvec).T
                
                # print "Camera X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(pos_camera[0], pos_camera[1], pos_camera[2],fps_detect)
                print ("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(tvec[0], tvec[1], tvec[2],self.fps_detect))

                if self.SHOW_FRAME:

                    #-- Print the tag position in camera frame
                    str_position = "MARKER Position x=%4.0f  y=%4.0f  z=%4.0f"%(tvec[0], tvec[1], tvec[2])
                    cv2.putText(frame, str_position, (0, 100), self.font, 1, (0, 0, 0), 2, cv2.LINE_AA)        
                    
                    #-- Print the marker's attitude respect to camera frame
                    str_attitude = "MARKER Attitude r=%4.0f  p=%4.0f  y=%4.0f"%(math.degrees(roll_marker),math.degrees(pitch_marker),
                                        math.degrees(yaw_marker))
                    cv2.putText(frame, str_attitude, (0, 150), self.font, 1, (0, 0, 0), 2, cv2.LINE_AA)

            else:
                print ("Nothing detected - fps = %.0f"%self.fps_read)
            

            if self.SHOW_FRAME:
                #--- Display the frame
                cv2.imshow('frame', frame)
         
        except Exception as e:
            print(e)
        cv2.waitKey(3)



    def main(self):

        rospy.init_node('image_listener')

        rospy.spin()
if __name__ == '__main__':
    cam=Camera()
    cam.main()
