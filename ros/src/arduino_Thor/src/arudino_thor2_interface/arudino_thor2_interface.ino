
//librerias necesarias para el funcionamiento
#include <Servo.h>
#include <Encoder.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

//mensajes y componenetes necesarios de ros
#include <ros.h>
#include <control_msgs/GripperCommand.h>
#include <arduino_Thor/joints_steps.h>
#include <arduino_Thor/sensors_raw.h>
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

// se crea el servo del efector final y la señal extra afuturo 
Servo efector;
#define analextra A0

// se definen los finales de carrera y a cuales pinesestan conectados 
#define limite56 6
#define limite3 15
#define limite2 14

// se indica el nombre de los potenciometros y se asigan a los pines analogicos 
#define pot4 A2
#define pot1 A4
int lecp4;
int lecp1;

// se crean los encoders del thor
Encoder Enc6(3, 2);
Encoder Enc5(18, 19);
Encoder Enc3(20, 23);
Encoder Enc2(21, 25);




// se crea el node handle
ros::NodeHandle  nh;

// se indica el mensaje a publicar y el tipo de mensaje al nodo
arduino_Thor::sensors_raw sensors;
ros::Publisher sensorData("sensor_raw_Thor2", &sensors );
 

//funcion que llama el nodo cuando recibe informacion de control de gripper
void servocmd( const control_msgs::GripperCommand& msg){

  efector.write(msg.position); 

}


//funcion que llama el nodo cuando recibe informacion de control de articulaciones
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


  sensors.joint_1= analogRead(pot1);
  sensors.joint_2= Enc2.read();
  sensors.joint_3= Enc3.read();
  sensors.joint_4= analogRead(pot4);
  sensors.joint_5= Enc5.read();
  sensors.joint_6= Enc6.read();

 
  sensorData.publish(&sensors );
  delay(5);

  pos[4] = joints.joint_5post;    //pulsos de art5 movimiento rotacion
  pos[5] = joints.joint_6post;  //pulsos de art6 movimiento rotacion

  motores.moveTo(pos);
  motores.runSpeedToPosition(); 
  
  sensors.joint_5= Enc5.read();
  sensors.joint_6= Enc6.read();
  
  sensorData.publish(&sensors );

}



// se indican los subcriptores del nodo, reciben la informacion para controlar el thor

ros::Subscriber<control_msgs::GripperCommand> sub("gripper_Thor2", &servocmd);
ros::Subscriber<arduino_Thor::joints_steps> arduino_joints("joints_steps_thor2", &arts);


void setup() {
  // put your setup code here, to run once:

  efector.attach(8);// se in dica a cual pin se conecta el servo
  // se indican los pines de finales de carrera como entradas 
  pinMode(limite56, INPUT);
  pinMode(limite3, INPUT);
  pinMode(limite2, INPUT);
  
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(arduino_joints);
  nh.advertise(sensorData);

  // Se añañaden los steppers al grupo de MultiStepper para que sea controlados
  motores.addStepper(Art1);
  motores.addStepper(Art2);
  motores.addStepper(Art3);
  motores.addStepper(Art4);
  motores.addStepper(Art5);
  motores.addStepper(Art6);

  //se ejecuta el home del thor
  Thor_Home();
  sensorData.publish(&sensors );
}

void loop() {
  // put your main code here, to run repeatedly:

  nh.spinOnce();
  delay(1);

}




void Thor_Home(){

  // Se configura la velocidad maxima de cada motor 
  Art1.setMaxSpeed(100);
  Art2.setMaxSpeed(100);
  Art3.setMaxSpeed(100);
  Art4.setMaxSpeed(100);
  Art5.setMaxSpeed(100);
  Art6.setMaxSpeed(100);


//====================================================
//========== Se ajusta la articulacion 2 =============
//====================================================
  int home2=-1; //variable para hacer home
  
 //Articulacion 2 se mueve hasta que el final de carrera optico se ponga en low
  while(digitalRead(limite2)){
    Art2.moveTo(home2);
    Art2.run();
    home2--;
    delay(1);
    }
    
  //Se pone la posicion actual del motor
  Art2.setCurrentPosition(0);
  // Se asignan los parametros del motor
  Art2.setMaxSpeed(100);
  Art2.setAcceleration(100);
  //se coloca el encoder en 0
  Enc2.write(0);

//====================================================
//========== Se ajusta la articulacion 3 =============
//====================================================
  int home3=1; //variable para hacer home
  
 //Articulacion 3 se mueve hasta que el final de carrera optico se ponga en low
  while(digitalRead(limite3)){
    Art3.moveTo(home3);
    Art3.run();
    home3++;
    delay(1);
    }
    
  //Se pone la posicion actual del motor
  Art3.setCurrentPosition(0);
  // Se asignan los parametros del motor
  Art3.setMaxSpeed(100);
  Art3.setAcceleration(100);
  //se coloca el encoder en 0
  Enc3.write(0);

//====================================================
//========== Se ajusta la articulacion 1 =============
//====================================================

//Para el brazo numero 1 el inicio o cero se encuntra en 
//Para el brazo numero 2 el inicio o cero se encuntra en 497
//por lo que se debe hacer el home a esa posicion respectiva

  int home1=0; //variable para hacer home
  while(analogRead(pot1)!=518){
    if(analogRead(pot1) > 518){
      home1--;
      Art1.moveTo(home1);
      Art1.run();
      delay(1); 
      }
     else{
      home1++;
      Art1.moveTo(home1);
      Art1.run();
      delay(1); 
     }
    }
  //Se pone la posicion actual del motor
  Art1.setCurrentPosition(0);
  // Se asignan los parametros del motor
  Art1.setMaxSpeed(100);
  Art1.setAcceleration(100);

  
//====================================================
//========== Se ajusta la articulacion 1 =============
//====================================================
//Para el brazo numero 1 el inicio o cero se encuntra en 
//Para el brazo numero 2 el inicio o cero se encuntra en 651
//por lo que se debe hacer el home a esa posicion respectiva

  int home4=0; //variable para hacer home
  while(analogRead(pot4)!=680){
    if(analogRead(pot4) > 680){
      home4++;
      Art4.moveTo(home4);
      Art4.run();
      delay(1); 
      }
     else{
      home4--;
      Art4.moveTo(home4);
      Art4.run();
      delay(1); 
     }
    }
  //Se pone la posicion actual del motor
  Art4.setCurrentPosition(0);
  // Se asignan los parametros del motor
  Art4.setMaxSpeed(100);
  Art4.setAcceleration(100);

  
//====================================================
//========= Se ajusta el sistema diferencial =========
//====================================================

  int home56=-1; //variable para hacer home
  long positions[6];
    
 //Articulacion 56 se mueve hasta que toque el final de carrera
  while(digitalRead(limite56)){
    Art5.moveTo(-home56);
    Art6.moveTo(home56);
    Art5.run();
    Art6.run();
    home56--;
    delay(1);
    }
 home56=1;
 //Articulacion 56 se devuelve hasta que deje de activar el final de carrera
  while(!digitalRead(limite56)){
    Art5.moveTo(-home56);
    Art6.moveTo(home56);
    Art5.run();
    Art6.run();
    home56++;
    delay(1);
    }

  //Se pone la posicion actual del motor
  Art5.setCurrentPosition(-127);
  Art6.setCurrentPosition(127);
  
  // Se asignan los parametros de los motores
  Art5.setMaxSpeed(100);
  Art6.setMaxSpeed(100);
  Art5.setAcceleration(100);
  Art6.setAcceleration(100);
  
  // se colocan los encoders a 90 grados
  Enc5.write(500);
  Enc6.write(-500); 

// se mueve los motores para que ponga en 0 el sistema diferencial
  positions[0] = 0;   //pulsos de art1
  positions[1] = 0;    //pulsos de art2
  positions[2] = 0;  //pulsos de art3
  positions[3] = 0;   //pulsos de art4
  positions[4] = 0;    //pulsos de art5
  positions[5] = 0;  //pulsos de art6
  motores.moveTo(positions);
  motores.runSpeedToPosition();

}
