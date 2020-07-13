#!/usr/bin/env python

import numpy as np
import sys
import rospy
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_matrix, euler_matrix
from ServicesDoorNavigation import *


def main(namespace=''):
    ## Inicializar nodo
    rospy.init_node('door_navigation_node')
    rospy.loginfo('Nodo /door_navigation_node iniciado')

    ## Frecuencia del nodo
    rate = rospy.Rate(1.0)

    ## Datos de la puerta tomados con el mapa del laboratorio
    doorMatrix = [
        [-1, 0, 0, 1.32],
        [0, -1, 0, -0.794],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

    ## De matriz a pose y calcular la posicion optima relativa a la puerta
    doorPose = geometry_msgs.msg.PoseStamped()
    doorPose.header.stamp = rospy.get_rostime()
    doorPose.pose = matrix2Pose(doorMatrix)
    # printPose(matrix2Pose(np.dot(doorMatrix, euler_matrix(0, 0, np.pi/2 + 50 * np.pi/180))))
    # M = np.dot(
    #     doorMatrix,
    #     euler_matrix(
    #            0,
    #            0,
    #            np.pi / 2 + loadParameter(namespace + 'door/ejepuerta/orientacion', 50 * np.pi / 180)
    #     )
    # )
    # printPose(matrix2Pose(M))
    M = np.dot(euler_matrix(0, 0, np.pi / 2 + loadParameter(namespace + 'door/ejepuerta/orientacion', 50 * np.pi / 180)),
               np.array([
                    [1, 0, 0, -loadParameter(namespace + 'door/ejepuerta/x', 1.12)],
                    [0, 1, 0, loadParameter(namespace + 'door/ejepuerta/y', 0)],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
               ]
           )
    )

    ## Hasta aqui todo ok

    ## Lectura de parametros de posicion optima del robot respecto de la puerta
    goalPose = geometry_msgs.msg.PoseStamped()
    goalPose.header.stamp = rospy.get_rostime()
    goalPose.pose = matrix2Pose(M)

    ## Llamada al servicio door_navigation
    print('Esperando al servicio ' + namespace + 'move_to_door...')
    rospy.wait_for_service(namespace + 'move_to_door')
    try:
        clientService = rospy.ServiceProxy(namespace + 'move_to_door', MoveToDoor)
        response = clientService(doorPose, goalPose, True)
        print('Goal pose from robot: ' + str(response.goal))
    except rospy.ROSInterruptException as e:
        print('Service call failed: %s' % e)


if __name__ == '__main__':
    try:
        ns = ''
        if len(sys.argv) > 1:
            ns = sys.argv[1]
            print('Namespace: ' + ns)
        main(ns)
    except rospy.ROSInterruptException:
        print('Program interrupted before completion')
