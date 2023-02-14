#pylint: disable-all
#!/usr/bin/env python3
# Echo client program
import socket
import rospy
from motion_plan.msg import Gripper_cmd
from std_srvs.srv import Trigger, TriggerRequest
import rospkg
import numpy as np


path = rospkg.RosPack().get_path('motion_plan')
SIMULATION=rospy.get_param('/unity_gripper_control/simulation')
class SecondOrderFilter():
    def __init__(self, size):
        self.filter_1 = np.zeros(size)
        self.filter_2 = np.zeros(size)
        self.dt = 0

    def initFilter(self, q, dt):
        self.filter_1 = np.copy(q)
        self.filter_2 = np.copy(q)
        self.dt = dt

    def filter(self, input, settling_time):
        gain = self.dt / (0.1 * settling_time + self.dt)
        self.filter_1 = (1 - gain) * self.filter_1 + gain * input
        self.filter_2 = (1 - gain) * self.filter_2 + gain * self.filter_1
        return self.filter_2


class GripperManager():
    def __init__(self, real_robot_flag = True, dt = 0.001, gripping_duration = 5.):
        self.gripping_duration = gripping_duration
        self.real_robot = real_robot_flag
        self.q_des_gripper = np.array([1.8, 1.8, 1.8])
        self.number_of_fingers = 3
        self.SO_filter = SecondOrderFilter(self.number_of_fingers)
        self.SO_filter.initFilter(self.q_des_gripper,dt)

    def resend_robot_program(self):
        rospy.sleep(1.5)
        rospy.wait_for_service("/ur_hardware_interface/resend_robot_program")
        sos_service = rospy.ServiceProxy('/ur_hardware_interface/resend_robot_program', Trigger)
        sos = TriggerRequest()
        result = sos_service(sos)
        # print(result)
        rospy.sleep(0.5)

    def mapToGripperJoints(self, diameter):
        return (diameter - 22) / (130 - 22) * (-np.pi) + np.pi  # D = 130-> q = 0, D = 22 -> q = 3.14

    def getDesGripperJoints(self):
        return self.SO_filter.filter(self.q_des_gripper, self.gripping_duration)

    def move_gripper(self, diameter =50, status = 'close'):
        # this is for the simulated robot, the diameter is converted into q for the fingers, that
        # will be appended to the desired joint published by the controller manager
        if not self.real_robot:
            q_finger = self.mapToGripperJoints(diameter)
            self.q_des_gripper = q_finger * np.ones(self.number_of_fingers)
            return

        # this is for the real robot, is a service call that sends a sting directly to the URcap driver
        import socket

        HOST = "192.168.0.100"  # The UR IP address
        PORT = 30002  # UR secondary client
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(0.5)
        try:
            sock.connect((HOST, PORT))
        except:
            raise Exception("Cannot connect to end-effector socket") from None
        sock.settimeout(None)
        scripts_path = rospkg.RosPack().get_path('motion_plan') + '/scripts'


        # 3 finger rigid gripper
        onrobot_script = scripts_path + "/onrobot_superminimal.script"
        file = open(onrobot_script, "rb")
        lines = file.readlines()
        file.close()

        tool_index = 0
        blocking = True
        cmd_string = f"tfg_release({diameter},  tool_index={tool_index}, blocking={blocking})"

        line_number_to_add = 446

        new_lines = lines[0:line_number_to_add]
        new_lines.insert(line_number_to_add + 1, str.encode(cmd_string))
        new_lines += lines[line_number_to_add::]

        offset = 0
        buffer = 2024
        file_to_send = b''.join(new_lines)

        if len(file_to_send) < buffer:
            buffer = len(file_to_send)
        data = file_to_send[0:buffer]
        while data:
            sock.send(data)
            offset += buffer
            if len(file_to_send) < offset + buffer:
                buffer = len(file_to_send) - offset
            data = file_to_send[offset:offset + buffer]
        sock.close()
        print("Gripper moved, now resend robot program")
        self.resend_robot_program()
        return
def callback(data):
    if(SIMULATION==False):
        #passing socket to callback
        print("here")
        g = GripperManager()
        if(data.close == True):
            g.move_gripper(diameter=70)
        else:
            g.move_gripper(diameter=100)

def listener():
    rospy.init_node("unity_gripper_control", anonymous=True)
    #seems to be that the only way to pass arguments to callback is using lambda func
    callback_lambda = lambda x: callback(x)
    rospy.Subscriber("/gripper_cmd", Gripper_cmd, callback_lambda,queue_size=10)
    rospy.spin()
    # spin() simply keeps python from exiting until this node is stopped
    
if __name__ == '__main__':
    print("\033[1m\033[96mGripper ready to receive command! :)\033[0m")
    with open(str(path+"/scripts/hello_world.txt"), 'r') as fin:
        print("\033[91m"+fin.read()+"\033[0m")
    if(SIMULATION==False):
        print("\033[1m\033[95mYou are in real-robot mode\033[0m")
    else:
        print("\033[1m\033[95mYou are in simulation mode\033[0m")
    listener()

