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


class imScale():

    def __init__(self, tempSup):

        # Pub
        self.image_pub_scaled = rospy.Publisher("/dados_sync/image_scaled", Image, queue_size = 10)
        self.image_pub_8bit = rospy.Publisher("/dados_sync/image_8bits", Image, queue_size = 10)
        self.pub_pc = rospy.Publisher("/dados_sync/point_cloud", PointCloud2, queue_size = 10)
        self.pub_odom = rospy.Publisher("/dados_sync/odometry", Odometry, queue_size = 10)

        # Sub
        self.thermal_sub = Subscriber("/termica/thermal/image_raw", Image)
        self.pc_sub = Subscriber("/zed/point_cloud/cloud_registered", PointCloud2)
        self.odom_sub = Subscriber("/zed/odom", Odometry)
        self.ats = ApproximateTimeSynchronizer([self.thermal_sub, self.pc_sub, self.odom_sub], queue_size=5, slop=0.02)
        self.ats.registerCallback(self.tcallback)
        self.bridge = CvBridge()
        
        self.tempFundoDeEscala = tempSup

    def tcallback(self, img_msg, pc_msg, odom_msg):

       cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="passthrough")


       ## Fator de ganho da camera termica
       #K = 0.04   #high resolution mode
       K = 0.4   #low resolution mode

       ## Escolhendo as temperaturas minimas e maximas a serem detectadas
       tempMin = 0  ;
       tempMax = self.tempFundoDeEscala;
       print "temperatura maxima: ", tempMax

       ## Escolhendo resolucao de temperatura
       resolucao = 5;

       ## Plotando imagem termica com escala fixa (tempMin --> tempMax)
       fig, ax = plt.subplots()
       fig.subplots_adjust(0,0,1,1)
       data = cv_image
       dataMin = (tempMin + 273.15)/K;
       dataMax = (tempMax + 273.15)/K;
       dataStep = resolucao/K;

       cax = ax.imshow(data, interpolation = 'none', cmap = 'jet_r', vmin = dataMin, vmax = dataMax)
       cbar = fig.colorbar(cax, ticks = np.arange(0, dataMax + dataStep, dataStep), orientation='vertical')
       cbar.ax.set_yticklabels(['{:.0f}'.format(x*K - 273.15) for x in np.arange(dataMin, dataMax+dataStep, dataStep)])
       ax.yaxis.set_major_locator(plt.NullLocator())
       ax.xaxis.set_major_locator(plt.NullLocator())
       fig.canvas.draw()

       ## Publicando imagem com escala acoplada
       data2 = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
       data2 = data2.reshape(fig.canvas.get_width_height()[::-1] + (3,))
       img_s_msg_out = self.bridge.cv2_to_imgmsg(data2, "bgr8")

       ## Publicando imagem termica pura
       data = cv_image
       m_dpi = 165;
       fig_raw, ax_raw = plt.subplots(figsize=(640.0/float(m_dpi) , 512.0/float(m_dpi)), dpi = m_dpi )
       fig_raw.subplots_adjust(0,0,1,1)
       plt.margins(0,0)
       ax_raw.set_axis_off()
       ax_raw.yaxis.set_major_locator(plt.NullLocator())
       ax_raw.xaxis.set_major_locator(plt.NullLocator())
       cax_raw = ax_raw.imshow(cv_image, interpolation = 'nearest', cmap = 'jet_r', aspect = 'auto', vmin = dataMin, vmax = dataMax)
       fig_raw.canvas.draw()
       data_raw = np.fromstring(fig_raw.canvas.tostring_rgb(), dtype=np.uint8, sep='')
       data_raw = data_raw.reshape(fig_raw.canvas.get_width_height()[::-1] + (3,))
       img_msg_out = self.bridge.cv2_to_imgmsg(data_raw, "bgr8");


       pc_msg_out = pc_msg
       odom_msg_out = odom_msg;
       t = rospy.Time.now()
       img_s_msg_out.header.stamp = t
       img_msg_out.header.stamp = t
       pc_msg_out.header.stamp = t
       odom_msg_out.header.stamp = t

       rospy.loginfo("publicando TERMICA")
       self.image_pub_scaled.publish(img_s_msg_out)
       self.image_pub_8bit.publish(img_msg_out)
       self.pub_pc.publish(pc_msg_out)
       self.pub_odom.publish(odom_msg_out)

       plt.close(fig) # MUITO IMPORTANTE PRA NAO ESTOURAR MEMORIA!
       plt.close(fig_raw)

       pass




def main(args):
    rospy.init_node('escala' , anonymous = True)
    tempSup = float(sys.argv[1]) # assim muda a escala de visualizacao

    imS = imScale(tempSup)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Terminando no"
    cv2.destroyAllWindows()


if __name__ == '__main__':
    print "Escala na imagem Termica ON"
    main(sys.argv)
