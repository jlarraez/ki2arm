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


# Funcion 'clear()' para limpiar la pantalla del terminal
def clear():
    os.system('clear')

if __name__ == '__main__':
    
    rospy.init_node('tf_kinect_listener_print')
    listener = tf.TransformListener()
	# Modificar segun ruta destino
    fo = open("/home/alumno/groovy_workspace/catkin_ws/src/ki2_arm/output/output.txt", "wb")

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans0,rot0) = listener.lookupTransform('/openni_depth_frame', '/right_shoulder', rospy.Time())
            (trans1,rot1) = listener.lookupTransform('/openni_depth_frame', '/right_elbow', rospy.Time())
            (trans2,rot2) = listener.lookupTransform('/openni_depth_frame', '/right_hand', rospy.Time())
        except (tf.LookupException, tf.ConnectivityException):
            continue


        ##################################################################
        ## Test de coordenadas '/openni_depth_frame', '/right_arm' 
        ##################################################################

        trans = ['trans x', 'trans y', 'trans z']
        rot = ['rot x', 'rot y', 'rot z', 'rot w']

        print '\n\n##Lectura /tf \t /right_shoulder \t/right_elbow \t\t/right_hand'

        for i in range(len(trans0)):
           print ' %s \t %0.5f \t\t' '%0.5f \t\t' '%0.5f \t\t' % (trans[i], trans0[i], trans1[i], trans2[i])

        print '\n'

        for j in range(len(rot0)):
            print ' %s \t\t %0.5f \t\t' '%0.5f \t\t' '%0.5f \t\t' % (rot[j], rot0[j], rot1[j], rot2[j])

        print '\n'


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

        print ' ## Test ang. \t /right_elbow \t\t/right_hand'
        for l in range(len(trans)):
           print ' %s \t %0.5f \t\t' '%0.5f \t\t' % (trans[l], v1[l], v2[l])

        print '\n'


        ##################################################################
        ## Test de angulos '/right_shoulder', '/right_arm' 
        ##################################################################
        # Coordenadas cilindricas (proyeccion en plano XY y)
        
        art1 = (math.acos(v3[1]/np.linalg.norm(v3)))-1.57
        art2 = (math.acos(v1[0]/np.linalg.norm(v1)))
        
        # Angulo formado entre v1 y v2
        art3 = math.acos(np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2)))
        print ' ## Test ang. \t /right_elbow \t\t/right_hand'
        print ' giro b \t %0.5f \t\t' '%0.5f \t\t' % (art1*(180/math.pi), art3*(180/math.pi))
        print ' giro a \t %0.5f \t\t'              % (art2*(180/math.pi))
        
        rate.sleep()
        clear()