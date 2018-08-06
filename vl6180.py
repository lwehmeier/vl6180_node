#!/usr/bin/python2
import math
from math import sin, cos, pi, radians
import rospy
import tf
from std_msgs.msg import Float32, Int32, Int16
import struct
from tca9548_driver.tca9548 import I2C_SW
from vl6180_driver.ST_VL6180X import VL6180X
import numpy as np
import time
VL_CHANNELS = {"lf" : 0x03, "lr" : 0x02, "rf": 0x04, "rr": 0x05, "fl": 0x06, "fr": 0x07}
DISTANCE_THRESHOLD =  200

class VL6180:
    def __init__(self, angle):
        self.distance = 255
        self.res = 0.1
        self.angle = angle
        tca.chn(VL_CHANNELS[angle])
        time.sleep(0.1)
        self.vl = VL6180X()
        self.vl.stop()
        time.sleep(0.01)
        self.vl.set_scale(1)
        self.vl.start()
        #self.vl.get_identification()
        #print(self.vl.idModel)
        #if not self.vl.ready:
        #    raise Exception("VL "+str(angle)+" failed to initialise")
        #self.vl.default_settings()
        #self.vl.get_distance()
    def update(self):
        angle = self.angle
        ch = VL_CHANNELS[angle]
        tca.chn(ch)
        time.sleep(0.1)
        self.distance = self.vl.get_distance()
        #print(self.distance)
        if self.distance < DISTANCE_THRESHOLD:
            self.interrupt()
    def interrupt(self):
        print("VL at "+ str(self.angle) +" detected obstacle. Stopping platform..")
        e_stop.publish(Int16(1))

tca = I2C_SW('I2C switch 0',0X70, 1)
vl_array = [
    VL6180("lf"),
    VL6180("lr"),
    VL6180("rf"),
    VL6180("rr"),
    VL6180("fl"),
    VL6180("fr")
#    VL6180("r_l"),# need more hardware
#    VL6180("r_r")
    ]

def callbackTimer(event):
    for vl in vl_array:
        vl.update()
        rospy.Publisher("/platform/distance/"+str(vl.angle), Int16, queue_size = 1).publish(Int16(vl.distance))

global cmap
cmap=None
rospy.init_node('vl6180')
e_stop = rospy.Publisher("/platform/e_stop", Int16, queue_size=1, tcp_nodelay=True)
rospy.Timer(rospy.Duration(0.5), callbackTimer)
r = rospy.Rate(0.5)
rospy.spin()
