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

# se establecen los mensajes
joint = joints_steps()
gripper= GripperCommand()


#=================================================
#======= FUNCION ENVIO DE PASOS AL ARDUINO =======
#=================================================


def get_joints(data): #funcion que se ejecuta cuando recibe la posicion de las articulaciones en radiades

    #variables globales que se modifican en esta funcion
    global ART1
    global ART2
    global ART3
    global ART4
    global ART5
    global ART6
    #verifica que el mensaje tenga informacion
    if len(data.position)==0:
        return
    #toma la informacion del mensaje, lo asigna a sus respectiva articulacion
    ART1=math.degrees(data.position[0])
    ART2=math.degrees(data.position[1])
    ART3=math.degrees(data.position[2])
    ART4=math.degrees(data.position[3])
    ART5=math.degrees(data.position[4])
    ART6=math.degrees(data.position[5])



def joints_steps_ref(data):

    global JointSteps1
    global JointSteps2
    global JointSteps3
    global JointSteps4
    global JointSteps5
    global JointSteps6

    JointSteps1= data.joint_1
    JointSteps2= data.joint_2
    JointSteps3= data.joint_3
    JointSteps4= data.joint_4
    JointSteps5= data.joint_5
    JointSteps6= data.joint_6


def sensor_joints(data):
    #funcion que recibe la posicion de los sensores
    dif1=0
    dif2=0
    dif3=0
    dif4=0
    dif5=0
    dif6=0

    Sensor_ART1=data.position[0]
    Sensor_ART2=data.position[1]
    Sensor_ART3=data.position[2]
    Sensor_ART4=data.position[3]
    Sensor_ART5=data.position[4]
    Sensor_ART6=data.position[5]


    #dif1=int(round((ART1-Sensor_ART1)*a1,0))
    if (ART2-Sensor_ART2)<-2 or 2<(ART2-Sensor_ART2):
        dif2=int(round((ART2-Sensor_ART2)*a2,0))

    if (ART3-Sensor_ART3)<-2 or 2<(ART3-Sensor_ART3):
        dif3=int(round((ART3-Sensor_ART3)*a3,0))
    #dif4=int(round((ART4-Sensor_ART4)*a4,0))
    if (ART5-Sensor_ART5)<-2 or 2<(ART5-Sensor_ART5):
        dif5=int(round((ART5-Sensor_ART5)*a5,0))
    if (ART6-Sensor_ART6)<-2 or 2<(ART6-Sensor_ART6):
        dif6=int(round((ART6-Sensor_ART6)*a6,0))



    joint.joint_1 = JointSteps1 + dif1
    joint.joint_2 = JointSteps2 + dif2
    joint.joint_3 = JointSteps3 + dif3
    joint.joint_4 = JointSteps4 + dif4
    joint.joint_5 = JointSteps5 + dif5
    joint.joint_6 = JointSteps6 + dif6

    # Finalemnte se publica el mensaje con todos los pasos para que el arduino los ejecute
    jointStep.publish(joint)
    #rospy.loginfo( [joint.joint_5, joint.joint_6] )

    rospy.loginfo( 'Pasos enviados al thor2 con exito' )


#=================================================
#=============== Funcion principal ===============
#=================================================



if __name__ == '__main__':

    # se inicia el nodo llamado Thor_arduino_actuadores
    rospy.init_node('Thor2_arduino_actuadores', anonymous=True)

    # se define jointStep para publicar en el topic  joints_steps
    jointStep = rospy.Publisher('joints_steps_thor2',joints_steps, queue_size=500)

    # se define grippercmd para pubicar en el topic gripper
    grippercmd = rospy.Publisher('gripper_thor2',GripperCommand, queue_size=100)

    #se define el subscriptor donde se obtienen las posiciones de las articualciones
    # leidas por
    rospy.Subscriber('steps_thor2_ref', joints_steps, joints_steps_ref )

    rospy.Subscriber('Sensor_joints_thor2', JointState, sensor_joints )

    rospy.Subscriber('/move_group/fake_controller_joint_states',
                                        JointState, get_joints )
    #para que el nodo se mantenga ejecutandose constnatemente
    rospy.spin()
