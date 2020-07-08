#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_euler, euler_from_matrix
from door_navigation.srv import *
import enum
from geometry_msgs.msg import Pose

class ErrorCode(enum.Enum):
    SUCCESS = 0
    ERROR_GET_ROBOT_POSITION = 1


def pose2Matrix(trans, rot):
    M = quaternion_matrix(rot)
    for index, item in enumerate(trans):
        M[index][-1] = item
    return M

def matrix2Pose(M):
    pose = Pose()
    pose.position.x = M[0][-1]
    pose.position.y = M[1][-1]
    pose.position.z = M[2][-1]
    rot = euler_from_matrix(M)
    q = quaternion_from_euler(rot[0], rot[1], rot[2])
    pose.orientation.x = q[0]
    pose.orientation.y = q[1]
    pose.orientation.z = q[2]
    pose.orientation.w = q[3]
    return pose



def MoveToDoorServer(request):
    ############################################################################
    ## MoveToDoorService
    ## Servicio de ROS para calcular la posicion optima de apertura de la puerta
    ## respecto del sistema del robot (/rb1_base_footprint).
    ## Una vez calculado se envía a /move_base para ejecute la navegacion.
    ############################################################################

    ## Inicializacion de la response del service
    response = MoveToDoorResponse
    response.errorCode = ErrorCode.SUCCESS

    ## Extraer la matriz de transformacion de la puerta
    doorMatrix = pose2Matrix(
        [request.doorPose.pose.position.x,
            request.doorPose.pose.position.y,
            request.doorPose.pose.position.z],
        [request.doorPose.pose.orientation.x, request.doorPose.pose.orientation.y,
            request.doorPose.pose.orientation.z, request.doorPose.pose.orientation.w]
    )

    ## Extraer la matriz de transformacion de la posicion optima en funcion de la puerta
    optimalPoseMatrix = pose2Matrix(
        [request.optimalPose.pose.position.x,
         request.optimalPose.pose.position.y,
         request.optimalPose.pose.position.z],
        [request.optimalPose.pose.orientation.x, request.optimalPose.pose.orientation.y,
         request.optimalPose.pose.orientation.z, request.optimalPose.pose.orientation.w]
    )

    ## Obtener la posicion del robot respecto del mapa
    try:
        (trans, rot) = tf.TransformListener().lookupTransform('/rb1_map', '/rb1_base_footprint', rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('Error al obtener la posicion del robot en el mapa...')
        response.errorCode = ErrorCode.ERROR_GET_ROBOT_POSITION
        response.ok = False
        return response

    ## Pasar la pose de trans+pos a matriz de transformacion
    rb1Matrix = pose2Matrix(trans, rot)

    ## Calculo de la pose optima respecto del rb1
    goalMatrix = np.dot(np.linalg.inv(rb1Matrix), doorMatrix)
    goalMatrix = np.dot(goalMatrix, optimalPoseMatrix)
    response.goal.pose = matrix2Pose(goalMatrix)
    response.goal.header.time = rospy.Time.now()
    pass

