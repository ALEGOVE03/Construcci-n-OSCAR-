#!/usr/bin/env python3
import rospy
import cv2 as cv
import numpy as np
import time

import tf2_ros
import tf2_geometry_msgs

from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose, Point, PointStamped
from tf.transformations import quaternion_from_euler



pose_obj= Pose()
bridge=CvBridge()
imagenPros=Image()
listo=False
rx=  0.4167    #0.440
ry=  0.4167    #0.587

def Processing(img):
    global listo
    img=  bridge.imgmsg_to_cv2(img)
    img2=np.copy(img)

    height, width,f = img.shape
    ox=int(width/2)
    oy=int(height/2)

    eje(img2,ox,oy)
    # Se realiza un filtrado media con un kernel de 5x5
    frame = cv.blur(img, (5,5))

    # Se realiza la segmentacion por color de las imagenes
    mask = segHSV(frame)

    # Se calcula el centro de los bloques y la rotacion
    box = calCentro(mask,img2)

    if listo:
        cent=box[0]
        #la rotacion de la caja va 0 a -90 la cual seria el yaw
        rotacion=-box[2]
        cx=int(round(cent[0],2))
        cy=int(round(cent[1],2))
        rospy.loginfo( box[1] )
        #a mm
        objx=(cx-ox)*rx
        objy=(cy-oy)*ry

        world_pointx=(550-objy)/1000
        world_pointy=(-objx)/1000
        quaternion=quaternion_from_euler(0,0,rotacion) #roll pitch yaw
        pose_obj.orientation.w= quaternion[3]
        pose_obj.orientation.x= quaternion[0]
        pose_obj.orientation.y= quaternion[1]
        pose_obj.orientation.z= quaternion[2]
        pose_obj.position.x=  world_pointx     #world_point[0]
        pose_obj.position.y=  world_pointy    #world_point[1]
        pose_obj.position.z= rotacion

        coordPub.publish(pose_obj)
        coords(cx,cy ,objx, objy, img2)

        imagenPros=bridge.cv2_to_imgmsg(img2)
        imgPub.publish(imagenPros)
        listo=False
    else:
        return None



# Función para realizar la segmentacion por medio de HSV
# devuelve las mascaras de los colores segmentados
def segHSV(img):

    HSV = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    # Se define el rango de azul a segmentar
    lower_blue = np.array([90, 60, 30])
    upper_blue = np.array([120, 255, 255])

    # Se define el rango de rojo a segmentar
    lower_red = np.array([0, 150, 70])
    upper_red = np.array([20, 255, 255])

    lower_red2 = np.array([170, 150, 70])
    upper_red2 = np.array([180, 255, 255])

    # Se define el rango de amarillo a segmentar
    lower_yellow = np.array([10, 140, 50])
    upper_yellow = np.array([35, 255, 255])

    # Se obtienen las mascaras de cada bloque
    red1= cv.inRange(HSV, lower_red, upper_red)
    red2= cv.inRange(HSV, lower_red2, upper_red2)
    mask1 = cv.inRange(HSV, lower_blue, upper_blue)
    mask2 = cv.add(red1,red2)
    mask3 = cv.inRange(HSV, lower_yellow, upper_yellow)

    return mask3+mask1


def calCentro(mask,img):
    # funcion para determinar el centro del objeto en una mascara
    global listo
    #se buscan los contornos, se espera que solo sea uno
    cnts, hier = cv.findContours(mask, cv.RETR_CCOMP,
                                    cv.CHAIN_APPROX_SIMPLE)
    if len(cnts)!=0:
        #se encierra el contorno en una caja
        rect=cv.minAreaRect(cnts[0])
        #se dibuja la caja para identificar el obajeto
        box= cv.boxPoints(rect)
        box=np.int0(box)
        cv.drawContours(img,[box],0,(0,255,0),2)

        #se devuelve la variable rect tal que
        #[(centrox,centroy),(ancho,alto),rotacion]
        listo=True
        return rect
    else:
        return None

def eje(img,ox,oy): #funcion que dibuja ejes

    #se dibuja el eje x
    cv.line(img,(ox,oy),(ox+100,oy),(0,0,255),4)
    #se dibuja el eje y
    cv.line(img,(ox,oy),(ox,oy+100),(0,255,0),4)
    return None


def coords( x, y, objx, objy ,img ):
    #funcion que  muestra las coordenadas en la imagene

    # Se imprimen las coordenadas del objeto en la imagen
    #se tienen los parametros para el texto
    font = cv.FONT_HERSHEY_SIMPLEX  # tipo de letra para impirmi
    xy=(x,y) # ubiacion coordenadas inferior izquierdas de las letras
    fontScale = 0.5  # escala de las letras
    color = (255, 0, 0) # color en BGR de las letras
    thickness = 1 # Anchura de las letras en pixeles
    cv.putText(img, str((round(objx,2),round(objy,2))) , xy, font,
                       fontScale, color, thickness, cv.LINE_AA)
    return None




if __name__ == '__main__':

    rospy.init_node('Thor_arduino', anonymous=True)

    #objt= rospy.Publisher('object_pose',joints_steps, queue_size=10)
    imgPub = rospy.Publisher('/camara/img_procesada',Image, queue_size=10)
    coordPub= rospy.Publisher('pose_objeto',Pose, queue_size=10)
    rospy.Subscriber('/camara/image_raw', Image ,Processing)
    rospy.spin()




rospy.Subscriber('/camara/image_raw', Image ,Processing)
