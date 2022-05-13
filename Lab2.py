"""
Allows to use the service dynamixel_command 
"""
import rospy
import time
# from std_msgs.msg import String
from dynamixel_workbench_msgs.srv import DynamixelCommand

from matplotlib.pyplot import get
import rospy
from geometry_msgs.msg import Twist
import termios, sys, os
from numpy import pi
TERMIOS = termios
t=1

__author__ = "F Gonzalez, S Realpe, JM Fajardo"
__credits__ = ["Felipe Gonzalez", "Sebastian Realpe", "Jose Manuel Fajardo", "Robotis"]
__email__ = "fegonzalezro@unal.edu.co"
__status__ = "Test"

from tkinter.tix import TCL_WINDOW_EVENTS, TixWidget
from unittest import case

def getkey():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    new = termios.tcgetattr(fd)
    new[3] = new[3] & ~TERMIOS.ICANON & ~TERMIOS.ECHO
    new[6][TERMIOS.VMIN] = 1
    new[6][TERMIOS.VTIME] = 0
    termios.tcsetattr(fd, TERMIOS.TCSANOW, new)
    c = None
    try:
        c = os.read(fd, 1)
    finally:
        termios.tcsetattr(fd, TERMIOS.TCSAFLUSH, old)
    return c
   
def jointCommand(command, id_num, addr_name, value, time):
    #rospy.init_node('joint_node', anonymous=False)
    rospy.wait_for_service('dynamixel_workbench/dynamixel_command')
    try:        
        dynamixel_command = rospy.ServiceProxy(
            '/dynamixel_workbench/dynamixel_command', DynamixelCommand)
        result = dynamixel_command(command,id_num,addr_name,value)
        rospy.sleep(time)
        return result.comm_result
    except rospy.ServiceException as exc:
        print(str(exc))

def check(tecla):
    id=1
    if(tecla == b'w'):
        id=id+1
        if(id==5):
            id=1

    if(tecla == b's'):
        id=id-1
        if(id<1):
            id=4

    if(tecla == b'a'):
        if(id==1):
            jointCommand('', 1, 'Goal_Position', 0, 0.5)
        if(id==2):
            jointCommand('', 2, 'Goal_Position', 2110, 0.5)
        if(id==3):
            jointCommand('', 3, 'Goal_Position', 2048, 0.5)
        if(id==4):
            jointCommand('', 4, 'Goal_Position', 2048, 0.5)
 
    if(tecla == b'd'):
        if(id==1):
            jointCommand('', 1, 'Goal_Position', 0, 0.5)
        if(id==2):
            jointCommand('', 2, 'Goal_Position', 2110, 0.5)
        if(id==3):
            jointCommand('', 3, 'Go(al_Position', 2048, 0.5)
        if(id==4):
            jointCommand('', 4, 'Goal_Position', 2048, 0.5)      
    print(id)
if __name__ == '__main__':

    try:
                
        jointCommand('', 1, 'Torque_Limit', 600, 0)
        jointCommand('', 2, 'Torque_Limit', 500, 0)
        jointCommand('', 3, 'Torque_Limit', 400, 0)
        jointCommand('', 4, 'Torque_Limit', 400, 0)
        id=1
        
        while(t):
            tecla=getkey()
            
            if(tecla == b'w'):
                id=id+1
                if(id==5):
                    id=1

            if(tecla == b's'):
                id=id-1
                if(id<1):
                    id=4

            if(tecla == b'a'):
                if(id==1):
                    jointCommand('', 1, 'Goal_Position', 0, 0.5)
                if(id==2):
                    jointCommand('', 2, 'Goal_Position', 2110, 0.5)
                if(id==3):
                    jointCommand('', 3, 'Goal_Position', 2048, 0.5)
                if(id==4):
                    jointCommand('', 4, 'Goal_Position', 2048, 0.5)
        
            if(tecla == b'd'):
                if(id==1):
                    jointCommand('', 1, 'Goal_Position', 300, 0.5)
                if(id==2):
                    jointCommand('', 2, 'Goal_Position', 2500, 0.5)
                if(id==3):
                    jointCommand('', 3, 'Goal_Position', 2500, 0.5)
                if(id==4):
                    jointCommand('', 4, 'Goal_Position', 2648, 0.5)      
                
            if(tecla==b'x'):
                t=0
            print(id)
    except rospy.ROSInterruptException:
        pass