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
    import urllib.request as request
    from pathlib import Path
    import json
    import xml.etree.ElementTree as etree
    import warnings

    def get_version(pkg_name): 
        version_str = str(subprocess.Popen(["rosversion", pkg_name], stdout=subprocess.PIPE).communicate()[0],'utf-8').replace("\n","").split(" ")
        if(version_str[0]=="Cannot"):
            return "0.0.0"
        else:
            return version_str[0]
    
    home=str(Path.home())
    warnings.simplefilter("ignore")
    print("Update Ubuntu")
    os.system("sudo apt-get update")
    print("Upgrade Ubuntu")
    os.system("sudo apt-get upgrade -y")
    print("Get paсkages list")
    update=0
    url="http://api.github.com/repos/IlyaDanilenko/geoscan_pioneer_max/contents/geoscan_ws/src"
    url_version="https://raw.githubusercontent.com/IlyaDanilenko/{}/{}/package.xml"
    package_list=json.loads(str(request.urlopen(url).read(),'utf-8'))
    for package in package_list:
        name=package['name']
        if package['size'] == 0:
            sha=package['sha']
            xml_text=etree.fromstring(str(request.urlopen(url_version.format(str(name),str(sha))).read(),'utf-8'))
            version=xml_text.getchildren()[1].text
            print("Check "+name+" paсkage")
            current_version=get_version(name)
            if current_version < version:
                update+=1
                print("\tUpdate "+name+" paсkage")
                subprocess.call("sudo rm -r "+home+"/geoscan_ws/src/"+name,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                subprocess.Popen(["git","clone","https://github.com/IlyaDanilenko/"+name+".git",home+"/geoscan_ws/src/"+name],stdout=subprocess.PIPE).communicate()
                subprocess.call(["cd",home+"/geoscan_ws/src/"+name,"&&","git checkout "+sha],shell=True)
                subprocess.call("sudo rm -r "+home+"/geoscan_ws/src/"+name+"/.git",shell=True)
            else:
                print("\tPaсkage "+name+" up to date")
    ext_msg=""
    if update > 0:
        print("Start rebuild workspace")
        os.system("cd "+str(Path.home())+"/geoscan_ws && catkin_make")
        ext_msg=" To update the environment run \'source ~/.bashrc\' or open a new terminal session."
    print("Updating Geoscan Pioneer Max system complite."+ext_msg)

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
    except Exception as e:
        print(str(e))
        pass
        
