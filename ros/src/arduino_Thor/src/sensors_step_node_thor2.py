#!/usr/bin/env python3

import math
import time
import rospy
from arduino_Thor.msg import sensors_raw
from sensor_msgs.msg import JointState

#=================================================
#============= DEFINICION DE VARIABLES ===========
#=================================================

# consntantes de la relacion entre lectura y grados
#permite la coneversion a grados
enc=0.18
enc_art56=0.09
pot1=0.704
pot4=1.76

pot1_or=500
pot4_or=680
#variables para hacer globlaes la posicion de los motores 5 y 6
Motor5=0
Motor6=0
Motor5_anterior = 0 # variables para hacer la diferencia de pasos
Motor6_anterior = 0


#variables auxiliares para hacer la conversion de radianes a pasos
ART1=0
ART2=0
ART3=0
ART4=0
ART5=0
ART6=0

# se establecen los mensajes
joints = JointState()

#=================================================
#==== FUNCIONES PARA LAS ARTICULACIONES 5 Y 6 ====
#=================================================

#funciones para obtener de la lectura global del enc5 y enc6
# el desplazamiento entre la lectura actual y la anterior para obtenerla
# en grados y acumularla para la articualcion 5
def ART5_position(M5,M6):
    M=(M5-Motor5_anterior)
    N=(M6-Motor6_anterior)
    if M > 0 and N < 0:
        A5=-(abs(M)+abs(N))*enc/2
        return A5

    if M < 0 and N >0:
        A5=(abs(M)+abs(N))*enc/2
        return A5
    return 0


#funciones para obtener de la lectura global del enc5 y enc6
# el desplazamiento entre la lectura actual y la anterior para obtenerla
# en grados y acumularla para la articualcion 6

def ART6_position(M5,M6):
    M=(M5-Motor5_anterior)
    N=(M6-Motor6_anterior)

    if M > 0 and N > 0:
        A6=(abs(M)+abs(N))*enc_art56/2
        return A6
    if M <  0 and N <  0:
        A6=-(abs(M)+abs(N))*enc_art56/2
        return A6
    return 0



#=======================================================
#======= FUNCION publica las lecturas procesadas =======
#=======================================================

def sensor_joints(data):
    #funcion que se ejecuta cuando recibe la posicion de las
    # articulaciones en radiades

    #variables globales que se modifican en esta funcion
    global Motor5
    global Motor6
    global Motor5_anterior
    global Motor6_anterior
    global ART5
    global ART6
    # toma la informacion del mensaje, lo asigna a sus respectiva
    # articulacion luego lo pasa a grados y lo multiplica por la
    # constante que pasa de grados a pasos
    joints.name=[6]
    joints.position=[6]

    ART1=(data.joint_1-pot1_or)*-pot1
    ART2=data.joint_2*-enc
    ART3=data.joint_3*enc
    ART4=(data.joint_4-pot4_or)*pot4


    Motor5=data.joint_5
    Motor6=data.joint_6

    ART5= ART5 + ART5_position(Motor5,Motor6)
    ART6= ART6 + ART6_position(Motor5,Motor6)

    Motor5_anterior= Motor5
    Motor6_anterior= Motor6


    joints.name=['joint1','joint2','joint3','joint4','joint5','joint6']

    joints.position=[ART1, ART2, ART3, ART4, ART5, ART6]

    # Finalemnte se publica el mensaje con todas las lecturas de
    # los sensores

    SensorJoint.publish(joints)
    #rospy.loginfo( [joint.joint_5, joint.joint_6] )



#=================================================
#=============== Funcion principal ===============
#=================================================



if __name__ == '__main__':

    # se inicia el nodo llamado Thor_arduino_sensores
    rospy.init_node('Thor2_arduino_sensores', anonymous=True)

    # se define jointStep para publicar en el topic  joints_steps
    SensorJoint = rospy.Publisher('Sensor_joints_thor2',JointState,
                                                    queue_size=500)

    # se define el subscriptor donde se obtienen las lecturas sin procesar
    # directamente del arduino del topic 'sensor_raw_Thor1' el cual ofrece
    # moveit
    rospy.Subscriber('sensor_raw_Thor2', sensors_raw, sensor_joints )

    #para que el nodo se mantenga ejecutandose constnatemente
    rospy.spin()
