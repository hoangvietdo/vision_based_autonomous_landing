import rospy, time, math, threading

from mavros_msgs.srv import CommandBool, SetMode, CommandBoolRequest, SetModeRequest
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped

class Px4Controller:
    def __init__(self):

        self.camera_x=0
        self.camera_y=0

        self.target_pose = PoseStamped()
        self.current_pose = PoseStamped()
        self.current_state = State()
        '''
        ros subscribers
        '''
        self.current_state_sub = rospy.Subscriber("/mavros/state", State, self.current_state_callback, queue_size = 10)
        self.camera_detection_sub = rospy.Subscriber("camera/detection", Float32MultiArray, self.detection_callback, queue_size = 10)
        self.current_pose_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
        '''
        ros publishers
        '''
        self.current_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.current_pose_callback, queue_size = 10)

        '''
        ros services
        '''
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)

        print("Px4 Controller Initialized!")

    def current_pose_callback(self, msg):
        self.current_pose = msg

    def current_state_callback(self, msg):
        self.current_state = msg

    def detection_callback(self,msg):
        self.detect=1
        x,y,z=msg.data
        self.camera_x=x
        self.camera_y=y

    def start(self):
        rospy.init_node("offboard_node", anonymous = True)
        rospy.loginfo("Node Initialized")
        self.rate = rospy.Rate(10)

        self.target_pose.header.stamp = rospy.Time.now()
        self.target_pose.pose.position.x = self.current_pose.pose.position.x
        self.target_pose.pose.position.y = self.current_pose.pose.position.y
        self.target_pose.pose.position.z = self.current_pose.pose.position.z 

        for i in range(100):
            self.current_pose_pub.publish(self.target_pose)
            self.rate.sleep()

        set_mode_req = SetModeRequest()
        set_mode_req.custom_mode = "OFFBOARD"

        arm_cmd_req = CommandBoolRequest()
        arm_cmd_req.value = True

        if self.flightModeService(set_mode_req).mode_sent:
            print("Offboard enabled")
        else:
            print("Offboard unenabled")
            pass

        if self.armService(arm_cmd_req).success:
            print("Armed")
        else:
            print("Arming Failed")
            pass
        '''
        main ROS thread
        '''
        for i in range(100):
            self.current_pose_pub.publish(self.target_pose)
            self.rate.sleep()

        while (rospy.is_shutdown() is False):
            self.target_pose.header.stamp = rospy.Time.now()
            self.target_pose.pose.position.x = self.current_pose.pose.position.x-self.camera_y
            self.target_pose.pose.position.y = self.current_pose.pose.position.y-self.camera_x
            self.target_pose.pose.position.z = self.current_pose.pose.position.z
           
            for i in range(10):
                self.current_pose_pub.publish(self.target_pose)
                self.rate.sleep()
           
            if self.x <= 0.1 and self.y <= 0.1 and self.x >= -0.1 and self.y >= -0.1:
                print("Landing")
                self.target_pose.header.stamp = rospy.Time.now()
                self.target_pose.pose.position.x = self.current_pose.pose.position.x
                self.target_pose.pose.position.y = self.current_pose.pose.position.y
                self.target_pose.pose.position.z = self.current_pose.pose.position.z - 0.1
                for i in range(10):
                    self.current_pose_pub.publish(self.target_pose)
                    self.rate.sleep()
           
            if (self.current_pose.pose.position.z < 0.05):
                arm_cmd_req.value = False
                if self.armService(arm_cmd_req).success:
                    print("Disarmed")
                else:
                    print("Disarm Failed")
                    pass

if __name__ == '__main__':
    con = Px4Controller()
    try:
        con.start()
    except rospy.ROSInterruptException:
        pass
