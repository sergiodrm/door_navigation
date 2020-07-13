#!/usr/bin/env python

import rospy
import tf
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_from_euler, euler_from_matrix
from door_navigation.srv import *
import enum
from geometry_msgs.msg import Pose
import actionlib
import move_base_msgs.msg

class ErrorCode(enum.Enum):
    SUCCESS = 0
    ERROR_GET_ROBOT_POSITION = 1

def printPose(pose):
    if type(pose) is type(geometry_msgs.msg.PoseStamped()):
        printPose(pose.pose)
    else:
        print('Position: [' + str([pose.position.x, pose.position.y, pose.position.z]))
        print('Orientation: [' + str([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]))

def pose2Matrix(trans, quat):
    M = quaternion_matrix(quat)
    for i in range(len(trans)):
        M[i][-1] = trans[i]
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

def loadParameter(name, defaultValue):
    if rospy.has_param(name):
        return rospy.get_param(name)
    else:
        print("Parametro " + name+ " no especificado, valor por defecto: " + str(defaultValue))
        return defaultValue

def MoveToDoorServer(request):
    ############################################################################
    ## MoveToDoorService
    ## Servicio de ROS para calcular la posicion optima de apertura de la puerta
    ## respecto del sistema del robot (/rb1_base_footprint).
    ## Una vez calculado se envia a /move_base para ejecute la navegacion.
    ############################################################################
    print('Ejecutando servicio /move_to_door')
    ## Inicializacion de la response del service
    response = MoveToDoorResponse()
    response.errorCode = ErrorCode.SUCCESS
    response.goal = geometry_msgs.msg.PoseStamped()

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
    print('Optimal pose')
    printPose(request.optimalPose)

    ## Multiplicar las matrices para obtener la posicion optima desde el rb1_map
    goalMatrix = np.dot(doorMatrix, optimalPoseMatrix)

    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = 'rb1_map'
    goal.header.stamp = rospy.Time.now()
    goal.pose = matrix2Pose(goalMatrix)
    printPose(goal)

    if request.execute:
        print('Iniciando ejecucion...')
        pub = rospy.Publisher('/rb1/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
        pub.publish(goal)

    return True, 0, goal


def main():
    ## Inicializar nodo
    rospy.init_node('door_service_navigation_node')
    rospy.loginfo('Nodo /door_service_navigation_node iniciado')

    ## Frecuencia del nodo
    rate = rospy.Rate(1.0)

    ## Inicializar el servicio de navegacion
    service = rospy.Service('move_to_door', MoveToDoor, MoveToDoorServer)
    print('Service /move_to_door ready!')
    # listener = tf.TransformListener()
    # while not rospy.is_shutdown():
    #    try:
    #        (trans, rot) = listener.lookupTransform('rb1_base_footprint', 'rb1_map', rospy.Time(0))
    #        print(str(trans))
    #        print(str(rot))
    #    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    #        print('Error')
    #        continue
    #
    #    rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print('Program interrupted before completion')