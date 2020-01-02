import rospy, time, math, threading

from pyquaternion import Quaternion

from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.srv import CommandBoolRequest, SetModeRequest
from mavros_msgs.msg import State

from geometry_msgs.msg import PoseStamped, Twist, Vector3Stamped
from std_msgs.msg import Float32, Float64, String,Float32MultiArray

class Px4Controller:
    def __init__(self):

        self.current_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.current_state = State()
        self.detect=0
        self.camera_x=0
        self.camera_y=0
        '''
        ros subscribers
        '''
        self.current_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_pose_callback, queue_size = 10)
        self.current_state_sub = rospy.Subscriber("/mavros/state", State, self.current_state_callback, queue_size = 10)
        self.camera_dection_sub = rospy.Subscriber("camera/detection", Float32MultiArray, self.detection_callback, queue_size = 10)

        '''
        ros publishers
        '''
        self.current_pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def detection_callback(self,msg):
        self.detect=1
        x,y,z=msg.data
        current_pose_camera_target = PoseStamped()
        current_pose_camera_target.header.stamp = rospy.Time.now()

        self.camera_x=x
        self.camera_y=y

        current_pose_camera_target.pose.position.x = self.current_pose.pose.position.x-y
        current_pose_camera_target.pose.position.y = self.current_pose.pose.position.y-x
        current_pose_camera_target.pose.position.z = self.current_pose.pose.position.z

        current_pose_pub.publish(current_pose_camera_target)
        self.rate.sleep()

        if x <= 1 and y<=1 and x >= -1 and y>=-1:
            current_pose_camera_target.pose.position.z = self.current_pose.pose.position.z - 0.2
            current_pose_pub.publish(current_pose_camera_target)
            self.rate.sleep()
            self.current_state = "LAND" 

    def current_state_callback(self, msg):
        self.current_state = msg

    def start(self):
        rospy.init_node("offboard_node", anonymous = True)
        rospy.loginfo("Node Initialized")
        self.rate = rospy.Rate(20)
        print("73")

        #while not rospy.is_shutdown() and self.current_state.connected:
        #    self.rate.sleep()

        print("78")
        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = 0.
        self.target_pose.pose.position.y = 0.
        self.target_pose.pose.position.z = 2.0

        for i in range(10):
            self.current_pose_pub.publish(self.target_pose)
            self.rate.sleep()
            print("87")

        set_mode_req = SetModeRequest()
        set_mode_req.custom_mode = "OFFBOARD"

        arm_cmd_req = CommandBoolRequest()
        arm_cmd_req.value = True

        if self.flightModeService(set_mode_req).mode_sent:
            print("Offboard enabled")
        else:
            pass
            #print("Offboard unenabled")

        print("101")

        if self.armService(arm_cmd_req).success:
            print("Armed")
        else:
            pass
            #print("Arming Failed")
        print("108")

        #self.current_pose_pub.publish(self.target_pose)
        #self.rate.sleep()
            
        '''
        main ROS thread
        '''

        #while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False):
        while (rospy.is_shutdown() is False):
            if self.detect is 0:
                print("Searching Marker")
                #searching
                self.target_pose.pose.position.x += 0.5
                self.current_pose_pub.publish(self.target_pose)
                time.sleep(3)
                self.rate.sleep()
                print("126")

                self.target_pose.pose.position.y += (-1)*(0.5)
                self.current_pose_pub.publish(self.target_pose)
                time.sleep(3)
                self.rate.sleep()
                print("132")
            
            #if (self.current_state is "LAND") and (self.current_pose.pose.position.z < 0.1):
            if (self.current_pose.pose.position.z < 0.1):
                self.current_state = "DISARMED"

if __name__ == '__main__':
    con = Px4Controller()
    try:
        con.start()
    except rospy.ROSInterruptException:
        pass
