#!/usr/bin/env python3

import rospy, socket, json, os, sys, subprocess, warnings, pickle, rosgraph
from rospy import ServiceProxy, Subscriber
from gs_interfaces.srv import Live
from rosservice import get_service_type
from std_msgs.msg import String
from gs_sensors import SensorManager
import urllib.request as request
from urllib.error import HTTPError
from pathlib import Path
import xml.etree.ElementTree as etree
from std_srvs.srv import Empty
sys.path.append("/home/ubuntu/geoscan_ws/src/gs_core/src/")
from restart import restart

def _check_master():
    try:
        rosgraph.Master("/rospioneer").getPid()
        return True
    except socket.error:
        return False

def _status():
    if _check_master():
        rospy.init_node("rospioneer", anonymous=True)
        if get_service_type("/geoscan/alive") == None:
            status = False
        else:
            status = ServiceProxy("geoscan/alive",Live)().status
        if status:
            sensors = SensorManager()
            altitude = sensors.altitude()
            roll, pitch, azimuth = sensors.orientation()
            charge,secs = sensors.power()
            print(
f"""Status:
    ONLINE
Battary:
    Time:{secs} secs.
    Charge: {charge} V.
Orientation:
    Roll: {roll} deg.
    Pitch: {pitch} deg.
    Azimuth: {azimuth} deg.
Altitude: {altitude} """)
        else:
            print("""Status:
            OFFLINE""")
    else:
        print("Unable to communicate with master!")

def _log_callback(data):
    print(f"\t{data.data}")

def _log():
    if _check_master():
        rospy.init_node("rospioneer",anonymous=True)
        print("Log messages:")
        Subscriber("geoscan/log",String,_log_callback)
        while not rospy.is_shutdown():
            pass
    else:
        print("Unable to communicate with master!")

def get_version(pkg_name): 
    version_str = str(subprocess.Popen(["rosversion", pkg_name], stdout=subprocess.PIPE).communicate()[0],'utf-8').replace("\n","").split(" ")
    if version_str[0]=="Cannot":
        return "0.0.0"
    else:
        return version_str[0]

def _update(argv):
    home = str(Path.home())
    warnings.simplefilter("ignore")
    print("\033[94mUpdate Ubuntu\033[00m")
    os.system("sudo apt-get update")
    print("\033[94mUpgrade Ubuntu\033[00m")
    os.system("sudo apt-get upgrade -y")
    print("\033[94mGet paсkages list\033[00m")
    update = 0
    branch = "master"
    repos ="http://api.github.com/repos/geoscan/geoscan_pioneer_max"
    url_branches=f"{repos}/branches"
    branches_list = json.loads(str(request.urlopen(url_branches).read(),'utf-8'))
    workspace_dir = "/geoscan_ws/src/"
    pickle_file = f"{home+workspace_dir}rospioneer/src/rospioneer/branch.pickle"
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
    print(f"\033[94mGit branch set to {branch}\033[00m")

    url = f"{repos}/contents/geoscan_ws/src?ref={branch}"
    url_version = "https://raw.githubusercontent.com/geoscan/{}/{}/package.xml"
    
    try:
        package_list = json.loads(str(request.urlopen(url).read(),'utf-8'))

        for package in package_list:
            name = package['name']
            if package['size'] == 0:
                sha = package['sha']
                xml_text = etree.fromstring(str(request.urlopen(url_version.format(str(name),str(sha))).read(),'utf-8'))
                version = xml_text.getchildren()[1].text
                print(f"Check {name} paсkage")
                current_version = get_version(name)
                if ( current_version < version ) or change_branch:
                    update += 1
                    print(f"\tUpdate {name} paсkage")
                    subprocess.call(f"sudo rm -r {home+workspace_dir+name}",shell=True,stdout=subprocess.PIPE, stderr=subprocess.PIPE)
                    subprocess.Popen(["git","clone",f"https://github.com/geoscan/{name}.git",home+workspace_dir+name],stdout=subprocess.PIPE).communicate()
                    subprocess.call(f"cd {home+workspace_dir+name} && git checkout {sha}",shell=True)
                    subprocess.call(f"sudo rm -r {home+workspace_dir+name}/.git",shell=True)
                else:
                    print(f"\tPaсkage {name} up to date")
        ext_msg = ""
        if update > 0:
            print("Start rebuild workspace")
            os.system(f"cd {home}/geoscan_ws && catkin_make")
            ext_msg = " To update the environment run \'source ~/.bashrc\' or open a new terminal session."
        print(f"\033[92mUpdating Geoscan Pioneer Max system complite.{ext_msg}\033[00m")
    except HTTPError:
        print(f"\033[91mgeoscan_ws/src not found in branch: {branch}\033[00m") 

def _start():
    subprocess.Popen(["roslaunch","gs_core","pioneer.launch","--screen"]).communicate()

def _camera():
    subprocess.Popen(["roslaunch","gs_camera","camera.launch","--screen"]).communicate()

def _restart():
    if _check_master():
        rospy.init_node("rospioneer",anonymous=True)
        if get_service_type("/geoscan/alive") != None:
            ServiceProxy("/geoscan/restart", Empty)()
        else:   
            restart()
    else:
        restart()

def _info():
    print("""rospioneer is comand-line tool for managing Geoscan Pioneer Max.
    
Command:
    \trospioneer start\tStart communication with Geoscan Pioneer board
    \trospioneer log    \tDisplays log messages
    \trospioneer status\tDisplays current state of Geoscan Pioneer Max
    \trospioneer camera\tLaunching broadcast from Raspberry Camera
    \trospioneer update\tUpdating all Geoscan Pioneer Max systems
    \trospioneer restart\tRestart  Geoscan Pioneer Base
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
        elif command == "restart":
            _restart()
        else:
            print("Wrong command. For a complete list of commands, enter \"rospioneer\"")
    except:
        pass