#!/usr/bin/env python3
# Echo client program
import socket
import rospy
from motion_plan.msg import Gripper_cmd
from std_srvs.srv import Trigger
import rospkg



SIMULATION=rospy.get_param('/unity_gripper_control/simulation')

path = rospkg.RosPack().get_path('motion_plan')


def callback(data,args):
    if(SIMULATION==False):
        s = args[0]
        rospy.wait_for_service('/ur_hardware_interface/resend_robot_program')
    script = ''
    if(data.close == False):
        script = path + '/scripts/open2.script'
        print("\033[96mOpening Gripper!\033[0m")
    elif(data.close == True):
        script = path + '/scripts/close.script'
        print("\033[96mClosing Gripper!\033[0m")
    else:
        print("Invalid argument!")
    if(SIMULATION==True):
        return
    f = open (script, "rb")
    l = f.read(4096)
    while (l):
        s.send(l)
        l = f.read(4096)
    f.close()
    s.close()
    rospy.sleep(1.5)
    
    #Resend robot program
    service = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)
    service()
    
    print("service called")
    rospy.sleep(1.5)

def listener(my_socket):
    rospy.init_node("unity_gripper_control", anonymous=True)
    #seems to be thath the only way to pass arguments to callback is using lambda func
    callback_lambda = lambda x: callback(x,my_socket)
    rospy.Subscriber("/gripper_cmd", Gripper_cmd, callback_lambda,queue_size=10)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    
def connect():
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect(("192.168.0.100", 30002))
    return s

if __name__ == '__main__':
    print("\033[1m\033[96mGripper ready to receive command! :)\033[0m")
    with open(str(path+"/scripts/hello_world.txt"), 'r') as fin:
        print("\033[91m"+fin.read()+"\033[0m")
    my_socket=None
    if(SIMULATION==False):
        print("\033[1m\033[95mYou are in real-robot mode\033[0m")
        my_socket = connect()
    else:
        print("\033[1m\033[95mYou are in simulation mode\033[0m")
    listener(my_socket)

