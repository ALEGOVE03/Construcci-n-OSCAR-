#!/usr/bin/env python3

import math
import time
import rospy
from control_msgs.msg import GripperCommand
from arduino_Thor.msg import joints_steps
from sensor_msgs.msg import JointState


#=================================================
#============= DEFINICION DE VARIABLES ===========
#=================================================

# consntantes de la relacion de engranes y pasos del motor
#permite la coneversion de grados a pasos
a1= 2.78
a2=17.14
a3=22.11
a4=1.11
a5=1.39
a6=2.78

#variables para hacer globlaes la posicion de los motores 5 y 6
Motor5=0
Motor6=0

#variables auxiliares para hacer la conversion de radianes a pasos
ART1=0
ART2=0
ART3=0
ART4=0
ART5=0
ART6=0
ART5_anterior = 0 # variables para hacer la diferencia de pasos
ART6_anterior = 0

# se establecen los mensajes
joint = joints_steps()
gripper= GripperCommand()



#=================================================
#==== FUNCIONES PARA LAS ARTICULACIONES 5 Y 6 ====
#=================================================

#funciones para obtener los pasos necesarios de la posicion actual
# a la deseada para la articualcion 5
def ART5_motor(j5):
    M=(j5-ART5_anterior)*a5

    M=int(round(M,0))
    return M

#funciones para obtener los pasos necesarios de la posicion actual
# a la deseada para la articualcion 6
def ART6_motor(j6):
    M=-(j6-ART6_anterior)*a6

    M=int(round(M,0))
    return M




#=================================================
#======= FUNCION ENVIO DE PASOS AL ARDUINO =======
#=================================================



def get_joints(data):
#funcion que se ejecuta cuando recibe la posicion de las articulaciones
#en radiades

    #variables globales que se modifican en esta funcion
    global Motor5
    global Motor6
    global ART5_anterior
    global ART6_anterior

    #verifica que el mensaje tenga informacion
    if len(data.position)==0:
        return
    if data.name!=['Thor1_Joint1','Thor1_Joint2','Thor1_Joint3',
                    'Thor1_Joint4','Thor1_Joint5','Thor1_Joint6']:
        return

    #toma la informacion del mensaje, lo asigna a sus respectiva
    # articulacion luego lo pasa a grados y lo multiplica por la
    # constante que pasa de grados a pasos

    ART1=math.degrees(data.position[0])*a1
    ART2=math.degrees(data.position[1])*-a2
    ART3=math.degrees(data.position[2])*-a3
    ART4=math.degrees(data.position[3])*-a4
    ART5=math.degrees(data.position[4])
    ART6=math.degrees(data.position[5])

    # cada articulacion en pasos se redondea y se vuelve un entero,
    #luego se asigna al menssaje joint para ser puplicado al arduino,
    #ademas se debe tomar en cuenta el comportamiento del sistema
    # diferencial de la articualcion 5 y 6, como du movmiento depende
    # de dos motores se hace de la forma mostrada, primero el movimiento
    #del 5 y luego del 6, ya que si se hace simultaneo no funciona

    joint.joint_1 = int(round(ART1,0))
    joint.joint_2 = int(round(ART2,0))
    joint.joint_3 = int(round(ART3,0))
    joint.joint_4 = int(round(ART4,0))

    # como el movimiento depende de los mismos dos motores se trabaja con
    #dos variables globales  y simplimente se suma o resta los pasos que
    #correspondan

    # Primero se ejecuta para la articulacion 5

    Motor5=Motor5 + ART5_motor(ART5)
    Motor6=Motor6 - ART5_motor(ART5)
    joint.joint_5 = Motor5
    joint.joint_6 = Motor6
    ART5_anterior=ART5

    #se vuleve a ejecutar para la articulacion 6
    Motor5=Motor5 + ART6_motor(ART6)
    Motor6=Motor6 + ART6_motor(ART6)
    joint.joint_5post = Motor5
    joint.joint_6post = Motor6
    ART6_anterior=ART6


    # Finalemnte se publica el mensaje con todos los pasos para
    # que el arduino los ejecute
    jointStep.publish(joint)
    rospy.loginfo( 'Pasos enviados al thor1 con exito' )



#=================================================
#=============== Funcion principal ===============
#=================================================



if __name__ == '__main__':

    # se inicia el nodo llamado Thor_arduino_actuadores
    rospy.init_node('Thor1_arduino_actuadores', anonymous=True)

    # se define jointStep para publicar en el topic  joints_steps
    jointStep = rospy.Publisher('joints_steps_thor1',joints_steps,
                                                        queue_size=500)

    # se define grippercmd para pubicar en el topic gripper
    grippercmd = rospy.Publisher('gripper_thor1',GripperCommand,
                                                        queue_size=100)

    # se define el subscriptor donde se obtienen las posiciones de las
    # articualciones del topic /move_group/fake_controller_joint_states
    # el cual ofrece moveit

    rospy.Subscriber('/move_group/fake_controller_joint_states',
                                                JointState, get_joints )

    #para que el nodo se mantenga ejecutandose constnatemente
    rospy.spin()
