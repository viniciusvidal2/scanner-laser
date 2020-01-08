#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Se inscreve no tópico 'thermal/image_raw'
# Cria colorbar e publica imagem no tópico 'thermal/image_scaled'
# Publica imagem em 8bits em escala termica (termica/thermal/image_8bit)

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import rospy
import sys, time
import numpy as np
import cv2
import roslib
#import matplotlib.pyplot as plt
from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from mpl_toolkits.axes_grid1 import make_axes_locatable
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Int8


class SalvarDados():

    def __init__(self):

        # Sub
        self.thermal_sub = Subscriber("/termica/thermal/image_raw", Image)
        self.odom_sub    = Subscriber("/zed/odom", Odometry)
        self.cloud_sub   = Subscriber("/zed/point_cloud/cloud_registered", PointCloud2) # NAO SEI AINDA SE E O CERTO
        self.ats = ApproximateTimeSynchronizer([self.thermal_sub, self.odom_sub, self.cloud_sub], queue_size=1, slop=0.2)
        self.ats.registerCallback(self.tcallback)
        self.bridge = CvBridge()
        
        self.tempFundoDeEscala = 50

        self.pasta = '/home/grin/Desktop/coleta'
        self.contador = 1

        f = open(self.pasta+"/odometrias.txt", "w+")

    def tcallback(self, ter_msg, odom_msg, cloud_msg):

       cv_image  = self.bridge.imgmsg_to_cv2(ter_msg, desired_encoding="passthrough")

       ## Fator de ganho da camera termica
       #K = 0.04   #high resolution mode
       K = 0.4   #low resolution mode

       ## Escolhendo as temperaturas minimas e maximas a serem detectadas
       tempMin = 0
       tempMax = self.tempFundoDeEscala



       pass




def main(args):
    rospy.init_node('salvando' , anonymous = False)

    sd = SalvarDados()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Terminando no"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print "Comecando no de gravacao"
    main(sys.argv)
