from tkinter.tix import TCL_WINDOW_EVENTS, TixWidget
from unittest import case

from matplotlib.pyplot import get
import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute, TeleportRelative
import termios, sys, os
from numpy import pi
TERMIOS = termios
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
def check(tecla):
    if(tecla == b'w'):
        pubVel(1,0,0,0.01)

    if(tecla == b's'):
        pubVel(-1,0,0,0.01)

    if(tecla == b'a'):
        pubVel(0,0,1,0.01)

    if(tecla == b'd'):
        pubVel(0,0,-1,0.01)

    if(tecla == b' '):
        teleportPI(0,-pi)

    if(tecla == b'r'):
        teleport(0,0,0)