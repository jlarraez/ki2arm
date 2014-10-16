#!/usr/bin/env python
############################################################################
#
#   Copyright (c) 2014
#   Pablo Melero Cazorla for Universidad de Almeria
#   
############################################################################

import roslib
roslib.load_manifest('ki2_arm')
import rospy
import math
import tf
import os
##
## Usamos la funcion np.sqrt() debido a su rapida respuesta de calculo
## 
## import timeit
## timeit.timeit('np.linalg.norm(x)', setup='import numpy as np; x = np.arange(100)', number=1000)
##      0.0450878
## timeit.timeit('np.sqrt(x.dot(x))', setup='import numpy as np; x = np.arange(100)', number=1000)
##      0.0181372
##
import numpy as np
from ki2_arm.msg import ArmPos

# Iniciamos el numero de secuencia para los mensajes a 0
sec=0

# Funcion 'clear()' para limpiar la pantalla del terminal
def clear():
    os.system('clear')

if __name__ == '__main__':
    
    pub = rospy.Publisher('arm_pos', ArmPos)

    rospy.init_node('tf_kinect_listener_pub')

    listener = tf.TransformListener()

    # Frecuencia de muestreo 20Hz
    rate = rospy.Rate(20.0)
    while not rospy.is_shutdown():
        try:
            (trans0,rot0) = listener.lookupTransform('/openni_depth_frame', '/right_shoulder', rospy.Time())
            (trans1,rot1) = listener.lookupTransform('/openni_depth_frame', '/right_elbow', rospy.Time())
            (trans2,rot2) = listener.lookupTransform('/openni_depth_frame', '/right_hand', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException):
            continue

        ##################################################################
        ## Test de coordenadas '/right_shoulder', '/right_arm' 
        ##################################################################

        # Vector0  /right_shoulder <-->  Origen
        # Vector1  /right_shoulder   ,   /right_elbow
        # Vector2  /right_shoulder   ,   /right_hand

        v1 = np.zeros(3)
        v2 = np.zeros(3)
        v3 = np.zeros(3)

        for k in range(len(v1)):            
            v1[k] = trans1[k] - trans0[k]

        for m in range(len(v2)):         
            v2[m] = trans2[m] - trans1[m]

        for s in range(len(v3)):         
            v3[s] = trans2[s] - trans0[s]

        ##################################################################
        ## Test de angulos '/right_shoulder', '/right_arm' 
        ##################################################################
        
        art1 = (math.acos(v3[1]/np.linalg.norm(v3)))-1.57
        art2 = (math.acos(v1[0]/np.linalg.norm(v1)))
        
        # Angulo formado entre v1 y v2
        art3 = math.acos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))

        # Resto de angulos a 0, ya que no es posible un control de ellos con un solo brazo
        # Posible control con el brazo izquierdo
        art4 = 0
        art5 = 0
        art6 = 0
      
        arm_angulos = [art1, art2, art3, art4, art5, art6]

        sec += 1
        pub.publish(sec, arm_angulos)

        rate.sleep()