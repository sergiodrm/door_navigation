#!/usr/bin/env python

import numpy as np
import sys
import rospy
import geometry_msgs.msg
import std_msgs.msg
import door_navigation.msg
import armcontrolmoveit.srv
from tf.transformations import quaternion_from_euler, quaternion_matrix, euler_from_matrix, euler_matrix
from ServicesDoorNavigation import *

InMotion = False

def InMotionCallback(data):
    global InMotion
    InMotion = data.data

def main(namespace='', runNav=False):
    ## Inicializar nodo
    rospy.init_node('door_navigation_node')
    rospy.loginfo('Nodo /door_navigation_node iniciado')
    ## Frecuencia del nodo
    rate = rospy.Rate(10)
    ## Inicializacion de publicaciones y suscripciones en topicos
    pubGoal = rospy.Publisher('/goalSJG', geometry_msgs.msg.PoseStamped, queue_size=10)
    pubError = rospy.Publisher('/door_navigation_error', door_navigation.msg.ErrorNavigation, queue_size=10)
    rospy.Subscriber('/rb1/robotnik_base_control/in_motion', std_msgs.msg.Bool, callback=InMotionCallback)

    ##########################################################################################################
    # Inicio del programa:
    #       1 -> crear parametro para indicar que la navegacion esta ejecutandose
    #       2 -> esperar a que el brazo y el torso esten en una posicion segura
    #       3 -> inicio de la navegacion
    #       4 -> cambio del parametro que indica la finalizacion de la navegacion
    ##########################################################################################################
    
    ########### 1 ###########
    ## Crear parametro para indicar el estado del nodo
    if not rospy.has_param(namespace + '/running_door_navigation'):
        rospy.set_param(namespace + '/running_door_navigation', True)


    ########### 2 ###########    
    ## Esperando a que el brazo se ponga en una posicion segura para la navegacion
    i = 0
    cad = ['/', '-', '\\', '|']
    securepos = rospy.get_param(namespace + '/secure_position_arm', default=False)
    while not securepos:
        print('Esperando a que el brazo se ponga en una configuracion segura ' + cad[i])
        sys.stdout.write("\033[F")
        i += 1
        if i >= len(cad):
            i = 0
        rate.sleep()
        securepos = rospy.get_param(namespace + '/secure_position_arm', default=False)

    
    ########### 3 ###########
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

    M = np.dot(
        euler_matrix(0, 0, np.pi / 2 + loadParameter(namespace + 'door/ejepuerta/orientacion', 50 * np.pi / 180)),
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
        response = clientService(doorPose, goalPose, runNav)
        pubGoal.publish(response.goal)
        rate.sleep()
    except rospy.ROSInterruptException as e:
        print('Service call failed: %s' % e)

    ## Hay que esperar hasta que el robot haya terminado la navegacion.
    ## Hay un topico que dice si el robot esta en movimiento o no ==>
    ## /rb1/robotnik_base_control/in_motion


    ## Una vez terminada la navegacion publicar, comprobar el error de posicion 
    ## antes de pasar a la etapa de apertura
    i = 0
    cad = ['/', '-', '\\', '|']
    while InMotion:
        print('Esperando a que el robot termine la navegacion ' + cad[i])
        sys.stdout.write("\033[F")
        i += 1
        if i >= len(cad):
            i = 0
        rate.sleep()

    ## Obtener el error de posicion respecto del punto esperado
    ## Para ello se crea un listener del nodo que publica las transformadas

    ## Lectura de posicion del robot
    listener = tf.TransformListener()
    trans, rot = [], []
    while len(trans) == 0 and len(rot) == 0:
        try:
            (trans, rot) = listener.lookupTransform('rb1_map', 'rb1_base_footprint', rospy.Time(0))
            print('Robot translation: ' + str(trans))
            print('Robot rotation: ' + str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    ## Calculo del error de posicion y de orientacion
    errorPos = [abs(trans[0] - response.goal.pose.position.x),
                abs(trans[1] - response.goal.pose.position.y),
                abs(trans[2] - response.goal.pose.position.z)]
    thetaGoal = euler_from_matrix(quaternion_matrix([
        response.goal.pose.orientation.x,
        response.goal.pose.orientation.y,
        response.goal.pose.orientation.z,
        response.goal.pose.orientation.w
    ]))
    thetaRobot = euler_from_matrix(quaternion_matrix(rot))
    errorRot = abs(thetaGoal[-1] - thetaRobot[-1])

    ## Mostrar errores por pantalla
    print('##########################################')
    print('* Error de posicion: ' + str(errorPos))
    print('* Modulo del error: ' + str(module(errorPos)))
    print('* Error de orientacion: ' + str(errorRot))
    print('##########################################')

    ## Publicando error en topico
    msg = door_navigation.msg.ErrorNavigation
    msg.errorPos = errorPos
    msg.errorRot = errorRot
    pubError.publish(msg)

    ########### 4 ###########
    print('Navegacion completada!')
    rospy.set_param(namespace + '/running_door_navigation', False)

    print('*** Fin del nodo door_navigation ***')


#############################################################################################################
#############################################################################################################

if __name__ == '__main__':

    ## Lectura de argumentos de entrada del programa
    ## - sys.argv[0]: nombre del programa
    ## - sys.argv[1]: ejecucion automatica de la navegacion
    ## - sys.argv[2]: espacio de nombres de ejecucion
    try:
        ns = ''
        autonav = False
        if len(sys.argv) > 2:
            ns = sys.argv[2]
            print('Namespace: ' + ns)
        if len(sys.argv) > 1:
            autonav = sys.argv[1] == 'true'
        print('Navegacion automatica: ' + str(autonav))
        main(ns, autonav)
    except rospy.ROSInterruptException:
        print('Program interrupted before completion')
