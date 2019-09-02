import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from sensor_msgs.msg import Imu, NavSatFix, Image
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2
import cv2.aruco as aruco
import sys, time, math


class Px4Controller:

    def __init__(self):

        self.imu = None
        self.gps = None
        self.local_pose = None
        self.current_state = None
        self.current_heading = None
        self.local_enu_position = None

        self.cur_target_pose = None
        self.cur_target_ati = None
        self.global_target = None
        self.thrust=0.58

        self.received_new_task = False
        self.arm_state = False
        self.offboard_state = False
        self.received_imu = False
        self.frame = "BODY"

        self.Pos_target_x=12.5
        self.Pos_target_y=-1
        self.Pos_target_z=5

        self.Ati_target_x=0
        self.Ati_target_y=0

        self.alti_err = 0
        self.pre_alti_err = 0
        self.alti_err_int = 0

        self.Pos_err_x = 0
        self.pr_Pos_err_x = 0
        self.Pos_err_int_x = 0
 
        self.Pos_err_y = 0
        self.pr_Pos_err_y = 0
        self.Pos_err_int_y = 0

        self.al_Kp = 0.035
        self.al_Ki = 0.000001
        self.al_Kd = 0.1

        self.cal_ati_x=0
        self.cal_ati_y=0

        self.state = None

        self.sta=0
        self.dt=0

        self.p_Kp = 1
        self.p_Ki = 0.000001
        self.p_Kd = 2

        self.bridge = CvBridge()
        self.camera_image=rospy.Subscriber("/gi/simulation/left/image_raw", Image, self.image_callback)
        self.id_to_find  = 0
        self.marker_size  = 1

        self.SHOW_FRAME  = True

        self.camera_matrix   = np.loadtxt("cameraMatrix.txt", delimiter=',')
        self.camera_distortion   = np.loadtxt("cameraDistortion.txt", delimiter=',')

        self.R_flip  = np.zeros((3,3), dtype=np.float32)
        self.R_flip[0,0] = 1.0
        self.R_flip[1,1] =-1.0
        self.R_flip[2,2] =-1.0

        self.aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.parameters  = aruco.DetectorParameters_create()

        self.font = cv2.FONT_HERSHEY_PLAIN

        self.t_read      = time.time()
        self.t_detect    = self.t_read
        self.fps_read    = 0.0
        self.fps_detect  = 0.0

        '''
        ros subscribers
        '''
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback)
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback)
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback)

        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback)
        self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)

        self.camera_image=rospy.Subscriber("/gi/simulation/left/image_raw", Image, self.image_callback)

        '''
        ros publishers
        '''
        self.attitude_target_pub = rospy.Publisher('mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def start(self):
        rospy.init_node("offboard_node")
	self.alti_err_int = 0
        self.cur_target_ati = self.construct_target_ati(self.Pos_target_x, self.Pos_target_y, self.Pos_target_z,self.thrust)
        print ("self.cur_target_ati:", self.cur_target_ati, type(self.cur_target_ati))

        for i in range(10):
            self.attitude_target_pub.publish(self.cur_target_ati)

            self.arm_state = self.arm()
            self.offboard_state = self.offboard()
            time.sleep(0.2)
            
        #if self.takeoff_detection():
        #    print("Vehicle Took Off!")

        #else:
        #    print("Vehicle Took Off Failed!")
        #    return

        '''
        main ROS thread
        '''

        while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
            self.attitude_target_pub.publish(self.cur_target_ati)

            #if (self.state is "LAND") and (self.local_pose.pose.position.z < 0.15):

            #    if(self.disarm()):

            #        self.state = "DISARMED"


            time.sleep(0.1)

    def construct_target_ati(self, x, y, z,thrust):
        target_raw_atti = AttitudeTarget()
        target_raw_atti.header.stamp = rospy.Time.now()

        target_raw_atti.body_rate.x = x*math.pi/180
        target_raw_atti.body_rate.y = y*math.pi/180
        target_raw_atti.body_rate.z = z*math.pi/180

        target_raw_atti.type_mask = AttitudeTarget.IGNORE_ATTITUDE 
        
        target_raw_atti.thrust = thrust

        return target_raw_atti

    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''
    def position_distance(self, cur_p, target_p, threshold=0.1):
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x)
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold):
            return True
        else:
            return False


    def local_pose_callback(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg
        self.alti_con()


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation)

        self.received_imu = True

    def gps_callback(self, msg):
        self.gps = msg

    def body2enu(self, body_target_x, body_target_y, body_target_z):

        ENU_x = body_target_y
        ENU_y = - body_target_x
        ENU_z = body_target_z

        return ENU_x, ENU_y, ENU_z


    def BodyOffsetENU2FLU(self, msg):

        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z


    def set_target_position_callback(self, msg):
        print("Received New Position Task!")

        if msg.header.frame_id == 'base_link':
            '''
            BODY_OFFSET_ENU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            FLU_x, FLU_y, FLU_z = self.BodyOffsetENU2FLU(msg)

            self.Pos_target_x = FLU_x + self.local_pose.pose.position.x
            self.Pos_target_y = FLU_y + self.local_pose.pose.position.y
            self.Pos_target_z = FLU_z + self.local_pose.pose.position.z
        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"

            self.Pos_target_x,self.Pos_target_y,self.Pos_target_z = self.body2enu(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)



    def position_PID(self):
        
        self.pr_Pos_err_x = self.Pos_err_x
        self.Pos_err_x = self.Pos_target_x - self.local_pose.pose.position.x
        self.Pos_err_int_x = self.Pos_err_int_x + self.Pos_err_x * self.dt

        self.pr_Pos_err_y = self.Pos_err_y
        self.Pos_err_y = self.Pos_target_y - self.local_pose.pose.position.y
        self.Pos_err_int_y = self.Pos_err_int_y + self.Pos_err_y * self.dt



        cal_ati_x  = (self.p_Kp * self.Pos_err_x) + (self.p_Ki * self.Pos_err_int_x) + (self.p_Kd * (self.Pos_err_x - self.pr_Pos_err_x) / self.dt)
        cal_ati_y  = (self.p_Kp * self.Pos_err_y) + (self.p_Ki * self.Pos_err_int_y) + (self.p_Kd * (self.Pos_err_y - self.pr_Pos_err_y) / self.dt)

        self.Ati_target_x=-cal_ati_y
        self.Ati_target_y=cal_ati_x
            

        if ((self.Ati_target_x > 15) or (self.Ati_target_x < -15)):
        
            self.Ati_target_x = 15 * (self.Ati_target_x / abs(self.Ati_target_x))
        

        if ((self.Ati_target_y > 15) or (self.Ati_target_y < -15)):
        
            self.Ati_target_y = 15 * (self.Ati_target_y / abs(self.Ati_target_y ))



    def alti_con(self):

        self.dt = self.get_dt()
        if self.dt<0.0001:
            return

        self.position_PID()

        self.pre_alti_err=self.alti_err
        self.alti_err=self.Pos_target_z-self.local_pose.pose.position.z
        self.alti_err_int =self.alti_err_int+ self.alti_err * self.dt

        self.thrust = 0.56 + (self.al_Kp * self.alti_err) + (self.al_Ki * self.alti_err_int) + (self.al_Kd * (self.alti_err - self.pre_alti_err) / self.dt)

        #print "P_err", self.Pos_err_x, self.Pos_err_y
        #print self.Pos_target_x, self.local_pose.pose.position.x
        #print self.alti_err, self.thrust

        if self.thrust>1:
            self.thrust=0.999

        self.cur_target_ati  = self.construct_target_ati(self.Ati_target_x,
                                                         self.Ati_target_y,
                                                         0,
                                                         self.thrust)

    def get_dt(self):
        dt=rospy.get_time () -self.sta
        self.sta=rospy.get_time ()
        return dt



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
            self.update_fps_read()
            
            #-- Convert in gray scale
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) #-- remember, OpenCV stores color images in Blue, Green, Red

            #-- Find all the aruco markers in the image
            corners, ids, rejected = aruco.detectMarkers(image=gray, dictionary=self.aruco_dict, parameters=self.parameters,
                                    cameraMatrix=self.camera_matrix, distCoeff=self.camera_distortion)
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
                
                #print "Camera X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f"%(pos_camera[0], pos_camera[1], pos_camera[2],fps_detect)
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


    '''
    Receive A Custom Activity
    '''
    
    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND":
            print("LANDING!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(0.1,
                                                         self.current_heading)

        if msg.data == "HOVER":
            print("HOVERING!")
            self.state = "HOVER"
            self.hover()

        if msg.data == "HOME":
            print("HOME")
            #self.state="HOME"
            self.Pos_target_x=self.home_x
            self.Pos_target_y=self.home_y


        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    def set_target_yaw_callback(self, msg):
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.z,
                                                     yaw_deg)

    '''
    return yaw from current IMU
    '''
    def q2yaw(self, q):
        if isinstance(q, Quaternion):
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z)
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad


    def arm(self):
        if self.armService(True):
            return True
        else:
            print("Vehicle arming failed!")
            return False

    def disarm(self):
        if self.armService(False):
            return True
        else:
            print("Vehicle disarming failed!")
            return False


    def offboard(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vechile Offboard failed")
            return False


    def hover(self):

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.z,
                                                     self.current_heading)

    def takeoff_detection(self):
        if self.local_pose.pose.position.z > 0.1 and self.offboard_state and self.arm_state:
            return True
        else:
            return False


if __name__ == '__main__':

    con = Px4Controller()
    con.start()

