#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from mavros_msgs.msg import *
from geographic_msgs.msg import *
from geometry_msgs.msg import *

#global variable
latitude = 0.0
longitude = 0.0

def call_set_mode(mode, mode_ID):
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        rospy.wait_for_service("/mavros/set_mode")
        print(service(mode_ID, mode))
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)

def pub_reset_gps():
    rospy.wait_for_service("/mavros/global_position/set_gp_origin")
    try:
        msg = GeoPointStamped()
        reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        reset_gps.publish(msg)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)

# TODO change to call_set_mode
#http://wiki.ros.org/mavros/CustomModes for custom modes
def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='GUIDED') #return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

# TODO change to call_set_mode        
def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
        isModeChanged = flightModeService(custom_mode='STABILIZE') #return true or false
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

def setLandMode():
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
        isLanding = landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " % e)
          
def setArm():
    pub_reset_gps()
    time.sleep(1)
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        # TODO this is to verify why the drone is arming but not launching
        rospy.set_param("/mavros/vision_pose/tf/listen", True)
        pub_reset_gps()

        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(True)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s"%e)
        
def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
        armService(False)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s"%e)


def setTakeoffMode():
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL) 
        takeoffService(altitude = 1, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def set_target_position(x,y,z,w):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.w = w

    try:
        set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)
        set_point_pub.publish(pose)
    except rospy.ServiceException as e:
        print("Service set_target_position call failed: %s" % e)

def pub_reset_gps():
    msg = GeoPointStamped()
    try:
        reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        reset_gps.publish(msg)
    except rospy.ServiceException as e:
        print("Service reset_gps call failed: %s" % e)

def pub_reset_gps():
    msg = GeoPointStamped()
    try:
        reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
        reset_gps.publish(msg)
    except rospy.ServiceException as e:
        print("Service reset_gps call failed: %s" % e)

def go_to_destination(dest = "2.8, 0.0, 2.0, 1.0"):
    x, y, z, w = dest.split(",")
    setGuidedMode()
    time.sleep(1)
    pub_reset_gps()
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    time.sleep(5)
    setArm()
    time.sleep(1)
    setTakeoffMode()
    time.sleep(5)
    set_target_position(x, y, z, w)
    time.sleep(1)
    setTakeoffMode()
    time.sleep(1)
    setDisarm()

def menu():
    print("Press")
    print("1: to set mode to GUIDED")
    print("2: to set mode to STABILIZE")
    print("3: to set mode to ARM the drone")
    print("4: to set mode to DISARM the drone")
    print("5: to set mode to TAKEOFF")
    print("6: to set mode to LAND")
    print("7: print GPS coordinates")
    print("8: Go to destination")
    print("9: fakegps")

def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7'])):
        menu()
        x = raw_input("Enter your input: ")
        if (x=='1'):
            setGuidedMode()
        elif(x=='2'):
            setStabilizeMode()
        elif(x=='3'):
            setArm()
        elif(x=='4'):
            setDisarm()
        elif(x=='5'):
            setTakeoffMode()
        elif(x=='6'):
            setLandMode()
        elif(x=='7'):
            global latitude
            global longitude
            print ("longitude: %.7f" %longitude)
            print ("latitude: %.7f" %latitude)
        elif(x=='8'):
            #dest = raw_input("Enter location to go : example 2.8, 0.0, 2.0, 2.0")
            dest = "1.0,0.0,1.0,0.3"
            go_to_destination(dest)
        elif(x=='9'):
            pub_reset_gps()
        else: 
            print("Exit")

def state_callback(data):
    rospy.rospy.loginfo("info Mode: %s" %data.mode)
    if data.mode == "MANUAL":
        #TODO review functionality
        rospy.loginfo("Control connected!!!")
    else:
        myLoop()


if __name__ == '__main__':
    try:
        rospy.init_node('uav_mission_planner', anonymous=True)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
        # state = rospy.Subscriber('/mavros/state', State, state_callback)
        # TODO check with subscription to state
        myLoop()
        # rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("#### Exception uav ####")

