#!/usr/bin/env python3

import rospy
import socket
import os
import sys
import subprocess

class ROSPioneerException(Exception):
    pass

def _check_master():
    import rosgraph
    try:
        rosgraph.Master("/rospioneer").getPid()
    except socket.error:
        raise ROSPioneerException("Unable to communicate with master!")

def _status():
    try:
        _check_master()
    except ROSPioneerException:
        print("Unable to communicate with master!")
        exit()
    if rospy.is_shutdown():
        return 
    rospy.init_node("rospioneer",anonymous=True)
    from rospy import ServiceProxy
    from gs_interfaces.srv import Live,Altitude,Orientation
    from rosservice import get_service_type
    if get_service_type("geoscan/alive") == None:
        status=False
    else:
        status=ServiceProxy("geoscan/alive",Live)().status
    if status:
        from gs_sensors import SensorManager
        altitude=ServiceProxy("geoscan/sensors/altitude_service",Altitude)().altitude
        orientation=ServiceProxy("geoscan/sensors/orientation_service",Orientation)
        current,charge,secs=SensorManager().power()
        print("""Status:
        ONLINE
        Battary:
        \tTime:{} secs.
        \tCurrent Voltage: {}
        \tCharge: {}%
        Orientation:
        \tRoll: {} deg.
        \tPitch: {} deg.
        \tAzimuth: {} deg.
        Altitude: {} h.""".format(secs,current,charge,orientation.roll,orientation.pitch,orientation.azimuth,altitude))
    else:
        print("""Status:
        OFFLINE""") 

def _log_callback(data):
    print("\t{}".format(data.data))

def _log():
    try:
        _check_master()
    except ROSPioneerException:
        print("Unable to communicate with master!")
        exit()
    if rospy.is_shutdown():
        return
    rospy.init_node("rospioneer",anonymous=True)
    from rospy import Subscriber
    from std_msgs.msg import String
    print("Log messages:")
    Subscriber("geoscan/log_topic",String,_log_callback)
    while not rospy.is_shutdown():
        pass

def _update():
    from pathlib import Path
    home=str(Path.home())
    print("Update Ubuntu")
    os.system("sudo apt-get update")
    print("Upgrade Ubuntu")
    os.system("sudo apt-get upgrade -y")
    print("Update packages")
    os.system("cd "+home+"/ && git submodule update")
    print("Build ROS workspace")
    os.system("cd "+home+"/geoscan_ws && catkin_make")
    print("Updating Geoscan Pioneer Max system complite")

def _start():
    subprocess.Popen(["roslaunch","gs_core","pioneer.launch","--screen"]).communicate()

def _camera():
    subprocess.Popen(["roslaunch","gs_camera","camera.launch","--screen"]).communicate()

def _info():
    print("""rospioneer is comand-line tool for managing Geoscan Pioneer Max.
    
Command:
    \trospioneer start\tStart communication with Geoscan Pioneer board
    \trospioneer log    \tDisplays log messages
    \trospioneer status\tDisplays current state of Geoscan Pioneer Max
    \trospioneer camera\tLaunching broadcast from Raspberry Camera
    \trospioneer update\tUpdating all Geoscan Pioneer Max systems
    """)
    exit()

def rospioneermain(argv=None):
    if argv is None:
        argv=sys.argv

    argv = rospy.myargv(argv)
    if len(argv) == 1:
        _info()
    try:
        command = argv[1]
        if command == "start":
            _start()
        elif command == "log":
            _log()
        elif command == "status":
            _status()
        elif command == "camera":
            _camera()
        elif command == "update":
            _update()
    except:
        pass
        
