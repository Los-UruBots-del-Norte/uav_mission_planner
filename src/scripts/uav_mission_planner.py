#!/usr/bin/env python
import time
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import State, SetMode, CommandBool, CommandTOL, GeoPointStamped, PoseStamped

#global variable
latitude = 0.0
longitude = 0.0

reset_gps = rospy.Publisher("/mavros/global_position/set_gp_origin", GeoPointStamped, queue_size=10)
set_point_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=10)

def mode_callback(data):
    if data.mode == "ACRO":
        call_set_mode("GUIDED", 4)
        # mission_planner()

def pub_reset_gps():
    msg = GeoPointStamped()
    reset_gps.publish(msg)

def set_target_position(x,y,z):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    set_point_pub.publish(pose)

### Service call functions ###
def call_set_mode(mode, mode_ID):
    print("### set_mode ###", mode, mode_ID)
    rospy.wait_for_service("/mavros/set_mode")
    try:
        service = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        service(base_mode=mode_ID, custom_mode=mode)
    except rospy.ServiceException as e:
        print('Service call failed: %s' % e)

def setGuidedMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        called = call_set_mode("GUIDED", 4)
        print("### setGuidedMode ###", called)
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled" % e)

def setStabilizeMode():
    rospy.wait_for_service('/mavros/set_mode')
    try:
        called = call_set_mode("STABILIZE", 0)
        print("### setStabilizeMode ###", called)
    except rospy.ServiceException as e:
        print("service set_mode call failed: %s. GUIDED Mode could not be set. Check that GPS is enabled"%e)

def setLandMode(altitude=0.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0):
    rospy.wait_for_service('/mavros/cmd/land')
    try:
        landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)
        isLanding = landService(altitude, latitude, longitude, min_pitch, yaw)
        print("### setLandMode ###", isLanding)
    except rospy.ServiceException as e:
        print("service land call failed: %s. The vehicle cannot land " % e)

def setArm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        called = armService(True)
        print("### setArm ###", called)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" % e)

def setDisarm():
    rospy.wait_for_service('/mavros/cmd/arming')
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        called = armService(False)
        print("### setDisarm ###", called)
    except rospy.ServiceException as e:
        print("Service arm call failed: %s" % e)

def setTakeoffMode(altitude=2.0, latitude=0.0, longitude=0.0, min_pitch=0.0, yaw=0.0):
    rospy.wait_for_service('/mavros/cmd/takeoff')
    try:
        takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        called = takeoffService(altitude, latitude, longitude, min_pitch, yaw)
        print("### setTakeoffMode ###", called)
    except rospy.ServiceException as e:
        print("Service takeoff call failed: %s" % e)

def globalPositionCallback(globalPositionCallback):
    global latitude
    global longitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    #print ("longitude: %.7f" %longitude)
    #print ("latitude: %.7f" %latitude)

def menu():
    print("Press")
    print("1: to set mode to GUIDED")
    print("2: to set mode to STABILIZE")
    print("3: to set mode to ARM the drone")
    print("4: to set mode to DISARM the drone")
    print("5: to set mode to TAKEOFF")
    print("6: to set mode to LAND")
    print("7: print GPS coordinates")
    print("8: mission planner")

def myLoop():
    x='1'
    while ((not rospy.is_shutdown())and (x in ['1','2','3','4','5','6','7'])):
        menu()
        x = input("Enter your input: ")
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
            mission_planner()
        else:
            print("Exit")

def goto_to_destination():
    print("### goto_to_destination ###")
    setTakeoffMode(2.5)
    time.sleep(10)
    set_target_position(2.8, 0.0, 2.0)
    time.sleep(20)
    setLandMode()
    time.sleep(10)

def mission_planner():
    print("### mission planner ###")
    call_set_mode("GUIDED", 4)
    time.sleep(1)
    rospy.set_param("/mavros/vision_pose/tf/listen", True)
    pub_reset_gps()
    time.sleep(5)
    setArm()
    time.sleep(5)
    goto_to_destination()

if __name__ == '__main__':
    try:
        rospy.init_node('uav_mission_planner', anonymous=True)
        rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
        # rospy.Subscriber("/mavros/state", State, mode_callback)
        myLoop()
        # rospy.spin()
    except rospy.ROSInterruptException:
        print("#### Exception uav ####")
