#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2014
#   Pablo Melero Cazorla for Universidad de Almeria
#   
############################################################################

import roslib
roslib.load_manifest('ki2_arm')
roslib.load_manifest('cob_script_server')
import rospy
import math
import tf
import os
import numpy as np
from ki2_arm.msg import ArmPos

from simple_script_server import *
sss = simple_script_server()

############################################################################
#       INICIO
#
# Introducir los datos pasados con los que se desea trabajar
# Y el numero de modulos o articulaciones

n_articulaciones = 3
m_datos_pasados = 5

############################################################################
#       MATRIZ ROTACIONES[nxm]

mat_hist = np.zeros([n_articulaciones,m_datos_pasados])

v_mov = np.zeros(3)

# Grado de variacion inima
grado_min = 5
rot_min = grado_min*(np.pi/180)


# Inicializamos el numero de mensajes recibidos en 0
msj_recibidos = 0

move = 0
move1 = 0

##################################################################
##      FUNCIONES


def callback(msg):

    global mat_hist
    global move
    global move1

    for l in reversed(range(n_articulaciones)): 

        for j in reversed(range(m_datos_pasados-1)):
            mat_hist[l,j+1]=mat_hist[l,j]

        mat_hist[l,0]=msg.angulos[l]

    #print msg.secuencia

    #detectar mov brazo
    if (abs(mat_hist[0,0]-mat_hist[0,1])>rot_min or abs(mat_hist[1,0]-mat_hist[1,1])>rot_min or abs(mat_hist[2,0]-mat_hist[2,1])>rot_min) and mat_hist[0,1]!=0:            
        #print 'brazo moviendose'
        move=0
        move1=1

    #mover brazo tras 2 
    if move>1 and move1==1:
        #print '\t \t\tMOVER BRAZO REAL'
        sss.move("arm", [[msg.angulos[0], msg.angulos[1], msg.angulos[2], 0, 0, 0]])
        move1=0

    move +=1


##   INTENTO DE BUCLE
#
#    for k in range(n_articulaciones):
#        if abs(mat_hist[k,0]-mat_hist[k,1])>rot_min:
#            print 'brazo moviendose'

    
def start():

    rospy.init_node('kinect2_arm')
    rospy.loginfo("Initializing all components...")
    sss.init("arm")
    #sss.move("arm","home") 
    rospy.Subscriber("arm_pos", ArmPos, callback, queue_size = 1)   
    rospy.spin()


##################################################################
##      MAIN

if __name__ == '__main__':

    while not rospy.is_shutdown():
        try:
            start()        
        except rospy.ROSInterruptException:
            pass