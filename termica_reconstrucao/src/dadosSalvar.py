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
        self.esq_sub = Subscriber("/stereo/left/image_raw", Image)
        self.dir_sub = Subscriber("/stereo/right/image_raw", Image)
        self.odom_sub = Subscriber("/stereo_odometer/odometry", Odometry)
        self.ats = ApproximateTimeSynchronizer([self.thermal_sub, self.esq_sub, self.dir_sub, self.odom_sub], queue_size=1, slop=0.2)
        self.ats.registerCallback(self.tcallback)
        self.bridge = CvBridge()
        
        self.tempFundoDeEscala = 50

        self.pasta = '/home/grin/Desktop/coleta'
        self.contador = 1

        f = open(self.pasta+"/odometrias.txt", "w+")

    def tcallback(self, ter_msg, esq_msg, dir_msg, odom_msg):

       cv_image  = self.bridge.imgmsg_to_cv2(ter_msg, desired_encoding="passthrough")
       esq_image = self.bridge.imgmsg_to_cv2(esq_msg, desired_encoding="bgr8")
       dir_image = self.bridge.imgmsg_to_cv2(dir_msg, desired_encoding="bgr8")

       ## Fator de ganho da camera termica
       #K = 0.04   #high resolution mode
       K = 0.4   #low resolution mode

       ## Escolhendo as temperaturas minimas e maximas a serem detectadas
       tempMin = 0
       tempMax = self.tempFundoDeEscala

       ## Escolhendo resolucao de temperatura
       resolucao = 5

       ## Plotando imagem termica com escala fixa (tempMin --> tempMax)
       fig, ax = plt.subplots()
       fig.subplots_adjust(0,0,1,1)
       data = cv_image
       dataMin = (tempMin + 273.15)/K
       dataMax = (tempMax + 273.15)/K
       dataStep = resolucao/K

       cax = ax.imshow(data, interpolation = 'none', cmap = 'jet_r', vmin = dataMin, vmax = dataMax)
       cbar = fig.colorbar(cax, ticks = np.arange(0, dataMax + dataStep, dataStep), orientation='vertical')
       cbar.ax.set_yticklabels(['{:.0f}'.format(x*K - 273.15) for x in np.arange(dataMin, dataMax+dataStep, dataStep)])
       ax.yaxis.set_major_locator(plt.NullLocator())
       ax.xaxis.set_major_locator(plt.NullLocator())
       fig.canvas.draw()

       data2 = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
       data2 = data2.reshape(fig.canvas.get_width_height()[::-1] + (3,))

       ## Publicando imagem termica pura
       data = cv_image
       m_dpi = 165
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

       ## Salvando dados de odometria no arquivo correto
       f = open(self.pasta+"/odometrias.txt", "a+")
       f.write("%d %.2f %.2f %.2f %.2f %.2f %.2f %.2f\n"%(self.contador, odom_msg.pose.pose.orientation.w, odom_msg.pose.pose.orientation.x, \
       odom_msg.pose.pose.orientation.y,  odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y, odom_msg.pose.pose.position.z))
       f.close()

       ## Escreve uma matriz com os valores de temperatura de uma vez para ler no matlab
    #    temps = np.array(data)
    #    for i in np.arange(data.shape[0]):
    #        for j in np.arange(data.shape[1]):
    #            temps[i, j] = data[i, j]*K - 273.15

       temps = data*K - 273.15*np.ones_like(data)

       ## Salvando todas as imagens como devem ser na pasta certa
       cv2.imwrite(self.pasta+'/ter_'+str(self.contador)+'.jpg', data_raw )
       cv2.imwrite(self.pasta+'/esq_'+str(self.contador)+'.jpg', esq_image)
       cv2.imwrite(self.pasta+'/dir_'+str(self.contador)+'.jpg', dir_image)
       cv2.imwrite(self.pasta+'/scl_'+str(self.contador)+'.jpg', data2    )
       cv2.imwrite(self.pasta+'/raw_'+str(self.contador)+'.jpg', temps    )

       rospy.loginfo('Salvando dados %d.', self.contador)

       self.contador = self.contador + 1

       plt.close(fig_raw)
       plt.close(fig)

       time.sleep(1)

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
