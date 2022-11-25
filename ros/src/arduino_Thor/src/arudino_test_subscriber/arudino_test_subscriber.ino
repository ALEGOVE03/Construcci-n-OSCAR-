
#include <Servo.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#include <ros.h>
#include <control_msgs/GripperCommand.h>
#include <arduino_Thor/joints_steps.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>


// Aca se nombran a todos los motores de las articulaciones, si bien hay 7 motores 
// ya que en la articuaclion se conenctan dos drivers 
AccelStepper Art1(AccelStepper::DRIVER, 47, 49); //step dir
AccelStepper Art2(AccelStepper::DRIVER, 43, 45); //step dir
AccelStepper Art3(AccelStepper::DRIVER, 39, 41); //step dir
AccelStepper Art4(AccelStepper::DRIVER, 35, 37); //step dir
AccelStepper Art5(AccelStepper::DRIVER, 31, 33); //step dir
AccelStepper Art6(AccelStepper::DRIVER, 27, 29); //step dir

// Se pueden colcoar hasta 10 steppers en el grupo MultiStepper
MultiStepper motores;

Servo myservo;

ros::NodeHandle  nh;


void messageCb( const control_msgs::GripperCommand& msg){

  myservo.write(msg.position); 

}



void arts( const arduino_Thor::joints_steps& joints){
  
  long pos[6]; // conjunto de posiciones que se desean alcanzar
   
  pos[0] = joints.joint_1;   //pulsos de art1
  pos[1] = joints.joint_2;    //pulsos de art2
  pos[2] = joints.joint_3;  //pulsos de art3
  pos[3] = joints.joint_4;   //pulsos de art4 
  
  pos[4] = joints.joint_5;    //pulsos de art5 movimiento traslacion 
  pos[5] = joints.joint_6;  //pulsos de art6 movimiento traslacion 
    
  motores.moveTo(pos);
  motores.runSpeedToPosition(); 


  pos[4] = joints.joint_5post;    //pulsos de art5 movimiento rotacion
  pos[5] = joints.joint_6post;  //pulsos de art6 movimiento rotacion

  motores.moveTo(pos);
  motores.runSpeedToPosition(); 

}


ros::Subscriber<control_msgs::GripperCommand> sub("gripper", &messageCb);

ros::Subscriber<arduino_Thor::joints_steps> arduino_joints("joints_steps", &arts);
 

void setup() {
  // put your setup code here, to run once:

  myservo.attach(9);
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(arduino_joints);

  
  // Se configura la velocidad maxima de cada motor 
  Art1.setMaxSpeed(100);
  Art2.setMaxSpeed(100);
  Art3.setMaxSpeed(100);
  Art4.setMaxSpeed(100);
  Art5.setMaxSpeed(100);
  Art6.setMaxSpeed(100);

  // Se añañaden los steppers al grupo de MultiStepper para que sea controlados
  motores.addStepper(Art1);
  motores.addStepper(Art2);
  motores.addStepper(Art3);
  motores.addStepper(Art4);
  motores.addStepper(Art5);
  motores.addStepper(Art6);

  
}

void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce();
  delay(1);

}
