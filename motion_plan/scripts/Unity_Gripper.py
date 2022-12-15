#!/usr/bin/env python 
# Echo client program
import socket
import sys
import os
import rospy
from motion_plan.msg import Gripper_cmd
import rospkg
HOST = "192.168.0.100" # The UR IP address
PORT = 30002 # UR secondary client



#f = open ("Grip.script", "rb")   #Robotiq Gripper
#f = open ("setzero.script", "rb")  #Robotiq FT sensor
path = rospkg.RosPack().get_path('motion_plan')
def callback(data):
    #s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    #s.connect((HOST, PORT))
    script= ''
    if(data.close == False):
        script = path + '/scripts/open1.script'
        print("\033[96mOpening Gripper!\033[0m")
    elif(data.close == True):
        script = path + '/scripts/close.script'
        print("\033[96mClosing Gripper!\033[0m")
    else:
        print("Invalid argument!")
    f = open (script, "rb")
    l = f.read(2024)
    while (l):
        #s.send(l)
        l = f.read(2024)
    f.close()
    #s.close()

def listener():
    rospy.init_node("unity_gripper_control", anonymous=True)
    rospy.Subscriber("/gripper_cmd", Gripper_cmd, callback,queue_size=10)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
if __name__ == '__main__':
    print("\033[1m\033[96mGripper ready to receive command! :)\033[0m")
    with open(str(path+"/scripts/hello_world.txt"), 'r') as fin:
        print("\033[91m"+fin.read()+"\033[0m")
    listener()

