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

def GoToPose(doorPose, goalPose, runNav=True, namespace=''):
    rate = rospy.Rate(0.5)
    pubGoal = rospy.Publisher('/goalSJG', geometry_msgs.msg.PoseStamped, queue_size=10)
    rate.sleep()
    ## Llamada al servicio door_navigation
    rospy.loginfo('Esperando al servicio ' + namespace + '/move_to_door...')
    rospy.wait_for_service(namespace + '/move_to_door')
    try:
        clientService = rospy.ServiceProxy(namespace + '/move_to_door', MoveToDoor)
        response = clientService(doorPose, goalPose, runNav)
        rate.sleep()
    except rospy.ROSInterruptException as e:
        rospy.logerr('Llamada al servicio fallida: %s' % e)
    

    ## Hay que esperar hasta que el robot haya terminado la navegacion.
    ## Hay un topico que dice si el robot esta en movimiento o no ==>
    ## /rb1/robotnik_base_control/in_motion
    pubGoal.publish(response.goal)

    ## Una vez terminada la navegacion publicar, comprobar el error de posicion 
    ## antes de pasar a la etapa de apertura
    while InMotion:
        rospy.loginfo('Esperando a que el robot termine la navegacion...')
        rate.sleep()

    return response.goal

def CalculateErrorNav(ref):
    ## Lectura de posicion del robot
    rate = rospy.Rate(1)
    listener = tf.TransformListener()
    trans, rot = [], []
    while len(trans) == 0 and len(rot) == 0:
        try:
            (trans, rot) = listener.lookupTransform('rb1_map', 'rb1_base_footprint', rospy.Time(0))
            # rospy.loginfo('Robot translation: ' + str(trans))
            # rospy.loginfo('Robot rotation: ' + str(rot))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        rate.sleep()

    ## Calculo del error de posicion y de orientacion
    errorPos = [abs(trans[0] - ref.pose.position.x),
                abs(trans[1] - ref.pose.position.y),
                abs(trans[2] - ref.pose.position.z)]
    thetaGoal = euler_from_matrix(quaternion_matrix([
        ref.pose.orientation.x,
        ref.pose.orientation.y,
        ref.pose.orientation.z,
        ref.pose.orientation.w
    ]))
    thetaRobot = euler_from_matrix(quaternion_matrix(rot))

    return errorPos, thetaGoal[-1] - thetaRobot[-1]

def main(namespace='', runNav=False):
    ## Inicializar nodo
    rospy.init_node('door_navigation_node')
    rospy.loginfo('Nodo /door_navigation_node iniciado')
    ## Frecuencia del nodo
    rate = rospy.Rate(0.5)
    ## Inicializacion de publicaciones y suscripciones en topicos
    
    pubError = rospy.Publisher('/door_navigation_error', door_navigation.msg.ErrorNavigation, queue_size=10)
    rospy.Subscriber('/rb1/robotnik_base_control/in_motion', std_msgs.msg.Bool, callback=InMotionCallback)
    pubGoal = rospy.Publisher(namespace + '/move_base_simple/goal', geometry_msgs.msg.PoseStamped, queue_size=10)
    pubTwist = rospy.Publisher(namespace + '/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)

    ##########################################################################################################
    # Inicio del programa:
    #       1 -> crear parametro para indicar que la navegacion esta ejecutandose
    #       2 -> esperar a que el brazo y el torso esten en una posicion segura
    #       3 -> inicio de la navegacion
    #       4 -> cambio del parametro que indica la finalizacion de la navegacion
    ##########################################################################################################
    
    ########### 1 ###########
    ## Crear parametro para indicar el estado del nodo
    if not rospy.has_param(namespace + '/running_nav'):
        rospy.set_param(namespace + '/running_nav', True)


    ########### 2 ###########    
    ## Esperando a que el brazo se ponga en una posicion segura para la navegacion
    securepos = rospy.get_param(namespace + '/secure_position_arm', default=False)
    while not securepos:
        rospy.loginfo('Esperando a que el brazo se ponga en una configuracion segura...')
        rate.sleep()
        securepos = rospy.get_param(namespace + '/secure_position_arm', default=False)
    rospy.loginfo('Brazo seguro para iniciar navegacion. Continuando...')

    
    ########### 3 ###########
    ## Datos de la puerta tomados con el mapa del laboratorio
    doorMatrix = [
        [-1, 0, 0, 1.32],
        [0, -1, 0, -0.754],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ]

    ## De matriz a pose y calcular la posicion optima relativa a la puerta
    doorPose = geometry_msgs.msg.PoseStamped()
    doorPose.header.stamp = rospy.get_rostime()
    doorPose.pose = matrix2Pose(doorMatrix)

    M = np.dot(
        euler_matrix(0, 0, np.pi / 2 + loadParameter(namespace + '/door/ejepuerta/orientacion', 50 * np.pi / 180)),
        np.array([
            [1, 0, 0, -loadParameter(namespace + '/door/ejepuerta/x', 1.12)],
            [0, 1, 0, loadParameter(namespace + '/door/ejepuerta/y', 0)],
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
    
    ## Mandar la posicion objetivo
    targetPose = GoToPose(doorPose, goalPose, namespace=namespace)

    ## Obtener el error de posicion respecto del punto esperado
    ## Para ello se crea un listener del nodo que publica las transformadas

    errorPos, errorRot = CalculateErrorNav(targetPose)

    ## Mostrar errores por pantalla
    rospy.loginfo('##########################################')
    rospy.loginfo('* Error de posicion: ' + str(errorPos))
    rospy.loginfo('* Modulo del error: ' + str(module(errorPos)))
    rospy.loginfo('* Error de orientacion: ' + str(errorRot))
    rospy.loginfo('##########################################')

    ## Publicando error en topico
    msg = door_navigation.msg.ErrorNavigation
    msg.errorPos = errorPos
    msg.errorRot = errorRot
    pubError.publish(msg)

    ########### 4 ###########
    if module(errorPos) > 0.05:
        rospy.logwarn('Navegacion con error de posicion excesivo (%f > 0.04)', module(errorPos))
        rospy.logwarn('Volviendo a ejecutar navegacion...')
        tw = geometry_msgs.msg.Twist()
        tw.linear.x = -0.2
        pubTwist.publish(tw)
        rate.sleep()
        pubTwist.publish(tw)
        rate.sleep()
        targetPose = GoToPose(doorPose, goalPose, namespace=namespace)
        errorPos, errorRot = CalculateErrorNav(targetPose)

        ## Mostrar errores por pantalla
        rospy.loginfo('##########################################')
        rospy.loginfo('* Error de posicion: ' + str(errorPos))
        rospy.loginfo('* Modulo del error: ' + str(module(errorPos)))
        rospy.loginfo('* Error de orientacion: ' + str(errorRot))
        rospy.loginfo('##########################################')

        ## Publicando error en topico
        msg = door_navigation.msg.ErrorNavigation
        msg.errorPos = errorPos
        msg.errorRot = errorRot
        pubError.publish(msg)

    ## Comprobar el error de orientacion, es muy importante que quede lo mas bajo posible
    ## Si no se puede corregir con planificaciones corregirlo manualmente en /rb1/cmd_vel.
    max_error_rot = 0.02
    if abs(errorRot) >= max_error_rot:
        rospy.logwarn('Demasiado error de rotacion. Corrigiendo manualmente...')
        ## Crear publisher en cmd_vel
        pubTwist = rospy.Publisher('/rb1/cmd_vel', geometry_msgs.msg.Twist, queue_size=10)
        rate.sleep() # Hay que esperar para que se cree el publisher en el master...

        ## Crear msg Twist. La rotacion sera proporcional al error (Regulador P)
        kp = 0.5
        command = geometry_msgs.msg.Twist()

        ## Crear un bucle que publique los comandos velocidad y lea el error para disminuir el error
        ## por debajo del limite.
        while abs(errorRot) >= max_error_rot:
            command.angular.z = kp * errorRot
            pubTwist.publish(command)
            rate.sleep()
            errorPos, errorRot = CalculateErrorNav(targetPose)
        rospy.loginfo('Error de orientacion corregido. Error actual: ' + str(errorRot))
    command.angular.z = -0.3
    pubTwist.publish(command)
    rate.sleep()
    pubTwist.publish(command)
    rate.sleep()
    
    rospy.loginfo('Navegacion completada!')
    rospy.set_param(namespace + '/running_nav', False)

    ## Fase 2: una vez abierta la puerta, girar la base para poder asegurar la posicion del brazo
    while not rospy.get_param(namespace + '/running_nav', default=False):
        rospy.loginfo('Esperando que termine el proceso de apertura para reiniciar la navegacion...')
        rate.sleep()

    rospy.loginfo('Iniciando navegacion para asegurar el brazo...')
    rospy.loginfo('Ejecutando giro de 90 grados...')
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = 'rb1_base_footprint'
    goal.header.stamp = rospy.Time.now()
    quat = quaternion_from_euler(0, 0, np.pi / 2)
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]
    pubGoal.publish(goal)

    while InMotion:
        rospy.loginfo('Esperando que termine la navegacion...')
        rate.sleep()
    
    rospy.loginfo('Navegacion terminada...')

    ## Indicar que se ha terminado la navegacion
    rospy.set_param(namespace + '/running_nav', False)

    ## Esperar a la posicion segura
    while not rospy.get_param(namespace + '/running_nav', default=False):
        rospy.loginfo('Esperando que termine el proceso de apertura para reiniciar la navegacion...')
        rate.sleep()

    ## Una vez que el brazo esta en posicion segura, se pide saber la pose del punto de apoyo para 
    ## realizar la navegacion a un punto que este en mitad del plano de la puerta alejado x distancia
    ## y mirando perpendicularmente al plano.

    srvName = '/door/support_position'
    try:
        rospy.loginfo("Esperando al servicio: %s" % srvName)
        rospy.wait_for_service(srvName)
        srv = rospy.ServiceProxy(srvName, armcontrolmoveit.srv.SupportPosition)
        res_getsupport = srv()
        rospy.loginfo('Llamada al servicio %s realizada correctamente!' % srvName)
    except rospy.ServiceException, e:
        rospy.logerr("Llamada a servicio %s fallida: %s" % (srvName, str(e)))
        return False
    
    ## Se pasa la posicion obtenida de pose a matriz
    M_apoyo = quaternion_matrix([
        res_getsupport.pose.orientation.x,
        res_getsupport.pose.orientation.y,
        res_getsupport.pose.orientation.z,
        res_getsupport.pose.orientation.w
    ])
    M_apoyo[0, 3] = res_getsupport.pose.position.x
    M_apoyo[1, 3] = res_getsupport.pose.position.y
    M_apoyo[2, 3] = res_getsupport.pose.position.z

    ## Traslacion sobre el eje Z local minimo 0.5 m (el radio de la base del RB-1)
    M_apoyo[2, 3] -= 0.7

    ## Sabiendo la rotacion que se le aplica a los sistemas de la puerta, se aplican rotaciones
    ## hasta que el sistema de la matriz se quede con el eje Z en la misma direccion que el eje Z
    ## del robot. De esta manera, la orientacion que tenga el plano XY en ese momento es la orientacion
    ## que debe coger el robot.

    ## Rotacion original: angulos de Euler ZYZ (90, 90, 90)
    M_goal = np.dot(M_apoyo, euler_matrix(-np.pi / 2, -np.pi / 2, 0, axes='szyz'))

    ## Pasar de la matriz a pose para enviarla a la navegacion
    rospy.loginfo('Navegando a posicion para empujar la puerta...')
    goal = geometry_msgs.msg.PoseStamped()
    goal.header.frame_id = 'rb1_base_footprint'
    goal.header.stamp = rospy.Time.now()
    quat = quaternion_from_euler(euler_from_matrix(M_goal))
    goal.pose.orientation.x = quat[0]
    goal.pose.orientation.y = quat[1]
    goal.pose.orientation.z = quat[2]
    goal.pose.orientation.w = quat[3]
    pubGoal.publish(goal)

    while InMotion:
        rospy.loginfo('Esperando que termine la navegacion...')
        rate.sleep()

    rospy.loginfo('Navegacion terminada...')

    ## Indicar que ha terminado la navegacion
    rospy.set_param(namespace + '/running_nav', False)

    rospy.loginfo('*** Fin del nodo door_navigation ***')


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
            rospy.loginfo('Namespace: ' + ns)
        if len(sys.argv) > 1:
            autonav = sys.argv[1] == 'true'
        rospy.loginfo('Navegacion automatica: ' + str(autonav))
        main(ns, autonav)
    except rospy.ROSInterruptException:
        rospy.logerr('Program interrupted before completion')
