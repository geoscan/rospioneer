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
    from gs_interfaces.srv import Live
    from rosservice import get_service_type
    if get_service_type("/geoscan/alive") == None:
        status = False
    else:
        status = ServiceProxy("geoscan/alive",Live)().status
    if status:
        from gs_sensors import SensorManager
        sensors = SensorManager()
        altitude = sensors.altitude()
        roll, pitch, azimuth = sensors.orientation()
        charge,secs = sensors.power()
        print(
"""Status:
    ONLINE
Battary:
    Time:{} secs.
    Charge: {} V.
Orientation:
    Roll: {} deg.
    Pitch: {} deg.
    Azimuth: {} deg.
Altitude: {} """.format(secs,charge,roll,pitch,azimuth,altitude))
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
    Subscriber("geoscan/log",String,_log_callback)
    while not rospy.is_shutdown():
        pass

def _update(argv):
    import urllib.request as request
    from urllib.error import HTTPError
    from pathlib import Path
    import json
    import xml.etree.ElementTree as etree
    import warnings
    import pickle

    def get_version(pkg_name): 
        version_str = str(subprocess.Popen(["rosversion", pkg_name], stdout=subprocess.PIPE).communicate()[0],'utf-8').replace("\n","").split(" ")
        if version_str[0]=="Cannot":
            return "0.0.0"
        else:
            return version_str[0]
    
    home = str(Path.home())
    warnings.simplefilter("ignore")
    print("\033[94m{}\033[00m" .format("Update Ubuntu"))
    os.system("sudo apt-get update")
    print("\033[94m{}\033[00m" .format("Upgrade Ubuntu"))
    os.system("sudo apt-get upgrade -y")
    print("\033[94m{}\033[00m" .format("Get paсkages list"))
    update = 0
    branch = "master"
    repos ="http://api.github.com/repos/geoscan/geoscan_pioneer_max"
    url_branches="{}/branches".format(repos)
    branches_list = json.loads(str(request.urlopen(url_branches).read(),'utf-8'))
    workspace_dir = "/geoscan_ws/src/"
    pickle_file = "{}rospioneer/src/rospioneer/branch.pickle".format(home+workspace_dir)
    try:
        with open(pickle_file,'rb') as f:
            branch = pickle.load(f)[0]
    except:
        pass
    change_branch = False
    if ( len(argv) >= 4 ) and ( ( argv[2] == "--branch" ) or ( argv[2] == "-b" ) ) and any(branch['name'] == argv[3] for branch in branches_list) and ( branch != argv[3] ):
        branch = argv[3]
        with open(pickle_file,'wb') as f:
            pickle.dump([branch],f)
        change_branch = True
    print("\033[94m{}\033[00m" .format("Git branch set to {}".format(branch)))

    url = "{}/contents/geoscan_ws/src?ref={}".format(repos,branch)
    url_version = "https://raw.githubusercontent.com/geoscan/{}/{}/package.xml"
    
    try:
        package_list = json.loads(str(request.urlopen(url).read(),'utf-8'))

        for package in package_list:
            name = package['name']
            if package['size'] == 0:
                sha = package['sha']
                xml_text = etree.fromstring(str(request.urlopen(url_version.format(str(name),str(sha))).read(),'utf-8'))
                version = xml_text.getchildren()[1].text
                print("Check {} paсkage".format(name))
                current_version = get_version(name)
                if ( current_version < version ) or change_branch:
                    update += 1
                    print("\tUpdate {} paсkage".format(name))
                    subprocess.call("sudo rm -r "+home+workspace_dir+name,shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    subprocess.Popen(["git","clone","https://github.com/geoscan/{}.git".format(name),home+workspace_dir+name],stdout=subprocess.PIPE).communicate()
                    subprocess.call("cd {} && git checkout {}".format(home+workspace_dir+name,sha),shell=True)
                    subprocess.call("sudo rm -r {}/.git".format(home+workspace_dir+name),shell=True)
                else:
                    print("\tPaсkage {} up to date".format(name))
        ext_msg = ""
        if update > 0:
            print("Start rebuild workspace")
            os.system("cd {}/geoscan_ws && catkin_make".format(home))
            ext_msg = " To update the environment run \'source ~/.bashrc\' or open a new terminal session."
        print("\033[92m{}\033[00m" .format("Updating Geoscan Pioneer Max system complite."+ext_msg))
    except HTTPError:
        print("\033[91m{}{}\033[00m".format("geoscan_ws/src not found in branch: ",branch)) 

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
        argv = sys.argv

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
            _update(argv)
    except:
        pass