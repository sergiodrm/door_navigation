#!/usr/bin/env python

import numpy as np
import rospy
import tf
from door_navigation.srv import *


def main():
    ## Inicializar nodo
    rospy.init_node('door_navigation')
    rospy.loginfo('Nodo /door_navigation iniciado')

    ## Frecuencia del nodo
    rate = rospy.Rate(1.0)

    ## Crear un listener del tf para conocer la posicion del robot
    listener = tf.TransformListener()
    while not rospy.is_shutdown():
        try:
            # listen to transform
            (trans, rot) = listener.lookupTransform('/rb1_map', '/rb1_base_footprint', rospy.Time(0))
            # print the transform
            rospy.loginfo('---------')
            rospy.loginfo('Translation: ' + str(trans))
            rospy.loginfo('Rotation: ' + str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        # sleep to control the node frequency
        rate.sleep()
    

if __name__ == '__main__':
    main()
