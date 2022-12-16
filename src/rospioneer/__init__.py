#!/usr/bin/env python3

import rospy, socket, json, os, sys, subprocess, warnings, pickle, rosgraph, signal
from rospy import ServiceProxy, Subscriber
from gs_vision.msg import QR, QR_array
from gs_interfaces.srv import Live
from gs_interfaces.srv import NavigationSystem
from geometry_msgs.msg import Point
from gs_interfaces.msg import SimpleBatteryState, PointGPS, OptVelocity, Orientation, SatellitesGPS, Parameter
from sensor_msgs.msg import Image
from rosservice import get_service_type
from std_msgs.msg import String, Int8, Float32
from gs_sensors import SensorManager
import urllib.request as request
from urllib.error import HTTPError
from pathlib import Path
import xml.etree.ElementTree as etree
from std_srvs.srv import Empty
from time import sleep
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
            status = ServiceProxy("geoscan/alive", Live)().status
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
            ServiceProxy("/geoscan/board/restart", Empty)()
        else:   
            restart()
    else:
        restart()
def _network(command):
    if command == "hotspot":
        os.system("sudo systemctl stop NetworkManager")
        os.system("sudo systemctl disable NetworkManager")
        os.system("sudo systemctl enable config-wlan")
        os.system("sudo systemctl start config-wlan")
        os.system("sudo systemctl enable isc-dhcp-server")
        os.system("sudo systemctl start isc-dhcp-server")
        os.system("sudo systemctl enable hostapd")
        os.system("sudo systemctl start hostapd")
        os.system("sudo systemctl restart code-server-wlan")
        os.system("sudo systemctl restart web-wlan-menu")
        os.system("sudo systemctl restart wlan-butterfly")
        os.system("sudo systemctl restart mission-control-wlan")
    elif command == "wifi":
        detector = subprocess.Popen(["roslaunch", "gs_vision", "qrcode_test.launch", "--screen"])
        while not _check_master():
            pass
        rospy.init_node("rospioneer",anonymous=True)
        while not rospy.is_shutdown():
            array = rospy.wait_for_message("geoscan/vision/qr", QR_array)
            network_data = {}
            if len(array.qrs) > 0:
                data = array.qrs[0].data.split(";")
                for d in data:
                    sub_data = d.split(":")
                    if sub_data[0] == "P":
                        network_data["password"] = sub_data[1]
                    elif sub_data[0] == "S":
                        network_data["ssid"] = sub_data[1]
                if network_data != {}:
                    detector.terminate()
                    os.system("sudo systemctl disable config-wlan")
                    os.system("sudo systemctl stop isc-dhcp-server")
                    os.system("sudo systemctl disable isc-dhcp-server")
                    os.system("sudo systemctl stop hosatpd")
                    os.system("sudo systemctl disable hostapd")
                    os.system("sudo systemctl enable NetworkManager")
                    os.system("sudo systemctl start NetworkManager")
                    os.system(f"sudo nmcli device wifi connect \"{network_data['ssid']}\" password {network_data['password']} name \"{network_data['ssid']}\"")
                    os.system("sudo systemctl restart code-server-wlan")
                    os.system("sudo systemctl restart web-wlan-menu")
                    os.system("sudo systemctl restart wlan-butterfly")
                    os.system("sudo systemctl restart mission-control-wlan")
                    exit()
            else:
                pass

def _info():
    print("""rospioneer is comand-line tool for managing Geoscan Pioneer Max.
    
Command:
    \trospioneer start\t\tStart communication with Geoscan Pioneer board
    \trospioneer log    \t\tDisplays log messages
    \trospioneer status\t\tDisplays current state of Geoscan Pioneer Max
    \trospioneer camera\t\tLaunching broadcast from Raspberry Camera
    \trospioneer update\t\tUpdating all Geoscan Pioneer Max systems
    \trospioneer restart\t\tRestart  Geoscan Pioneer Base
    \trospioneer network hotspot\tPuts the Raspberry Pi into hotspot mode
    \trospioneer network wifi\t\tConnects Raspberry Pi to wi-fi network shown on QR code
    \trospioneer selfcheck \t\t Check Geoscan Pioneer Max system
    """)
    exit()

def _check_topic(topic_name, topic_type):
    try:
        rospy.wait_for_message(topic_name, topic_type, 5)
        return True
    except rospy.ROSException:
        return False

def _check_service(service_name, service_type):
    msg = None
    if get_service_type(service_name) is not None:
        msg = ServiceProxy(service_name, service_type)()
    return msg

def _selfcheck():
    check_topic_dict = {
        "/geoscan/battery_state" : SimpleBatteryState,
        "/geoscan/navigation/local/position" : Point,
        "/geoscan/navigation/local/velocity" : Point,
        "/geoscan/navigation/local/yaw" : Float32,
        "/geoscan/navigation/global/position" : PointGPS,
        "/geoscan/navigation/global/status" : Int8,
        "/geoscan/navigation/opt/velocity" : OptVelocity,
        "/geoscan/navigation/satellites" : SatellitesGPS,
        "/geoscan/sensors/gyro" : Point,
        "/geoscan/sensors/accel" : Point,
        "/geoscan/sensors/altitude" : Float32,
        "/geoscan/sensors/gyro" : Point,
        "/geoscan/sensors/mag" : Point,
        "/geoscan/sensors/orientation" : Orientation
    }

    print("Start checking")
    print("ROS core checking...")
    process = subprocess.Popen(["roslaunch","gs_core","pioneer.launch"], stdout=subprocess.PIPE, preexec_fn=os.setsid)
    roscore = False
    sleep(5)
    for _ in range(10):
        roscore = _check_master()
    if roscore:
        print(f"\033[92mROS core check - OK\033[00m")
        print("Live Status checking...")
        sleep(5)
        status = False
        rospy.init_node("rospioneer", anonymous=True)
        for _ in range(5):
            if get_service_type("/geoscan/alive") is None:
                status = False
            else:
                status = ServiceProxy("geoscan/alive", Live)().status

        if status:
            print(f"\033[92mLive Status check - OK\033[00m")
        else:
            print("\033[91mLive Status check - FAILE \033[00m")

        print("=======================")
        print("Checking topics...")
        
        for name in check_topic_dict.keys():
            if _check_topic(name, check_topic_dict[name]):
                print(f"\033[92m{name} check - OK\033[00m")
            else:
                print(f"\033[91m{name} - FAILE \033[00m")

        print("=======================")
        print("Checking service...")

        system = _check_service("/geoscan/navigation/get_system", NavigationSystem)
        if system is not None:
            print(f"\033[92mNavigation system - {system.navigation}\033[00m")
        else:
            print(f"\033[91mNavigation system - FAILE \033[00m")

    else:
        print("\033[91mROS core check - FAILE \033[00m")
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)

    print("=======================")
    print("Checking camera...")

    process = subprocess.Popen(["roslaunch","gs_camera","camera.launch"], stdout=subprocess.PIPE, preexec_fn=os.setsid)
    camera = False
    sleep(10)

    if not roscore:
        rospy.init_node("rospioneer", anonymous=True)

    for _ in range(5):
        camera = _check_topic("/pioneer_max_camera/image_raw", Image)
    
    if camera:
        print(f"\033[92mCamera check - OK\033[00m")
    else:
        print(f"\033[91mCamera check - FAILE \033[00m")
    
    os.killpg(os.getpgid(process.pid), signal.SIGTERM)
    print("Finish check")

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
        elif command == "network":
            _network(argv[2])
        elif command == "selfcheck":
            _selfcheck()
        else:
            print("Wrong command. For a complete list of commands, enter \"rospioneer\"")
    except Exception as e:
        print(str(e))