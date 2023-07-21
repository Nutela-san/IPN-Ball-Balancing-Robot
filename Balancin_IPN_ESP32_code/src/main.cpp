#include <Arduino.h>
#include <InterCom.h>
#include <ServoController.h>
#include <IK_3RPS.h>

//#define CALIBRATION  //Descomentar cuando se desee realizar la calibracion de los servos

//------------ DEFINICION DE PINES -----------------
const uint8_t pines_servos[3]= {13,12,14};
//--------------------------------------------------

//------------ DECLARACION DE OBJETOS -----------------
SimpleCommand cmd;
ServoControl  servo1(0,pines_servos[0],14,50), servo2(1,pines_servos[1],14,50), servo3(2,pines_servos[2],14,50);
            //servo#(pwm_chanel, pin, resolution_bits, freq)
const double in2mm = 25.40;
const double deg2rads = (PI/180);
MachineIK robot_IK(2*in2mm,3*in2mm,1.75*in2mm,3*in2mm);
//--------------------------------------------------

#ifdef CALIBRATION

ServoControl servos[3] = {servo1,servo2,servo3};
unsigned long last_time = 0;
float pos = 90, last_pos = 90;
bool test = false;
uint8_t seq = 0; //para secuencia de prueba
float servo_select = 1, last_servo = 1;
float pwm_range[2] = {servo1.pwm_range[0],servo1.pwm_range[1]};
float last_pwm_range[2] = {pwm_range[0],pwm_range[1]};

void list(){
  cmd.list();
}

void popo(){
  servo1.print_info_servo();
}

void doTest(){
  test = !test;
}

void command_config(){
  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("test",doTest);
  cmd.addCommand("servo",&servo_select);
  cmd.addCommand("min",&pwm_range[0]);
  cmd.addCommand("max",&pwm_range[1]);
  cmd.addCommand("p",&pos);

  cmd.begin(115200);
}

void setup() {
  for(uint8_t i=0;i<3;i++){
    servos[i].begin();
  }

  command_config();

  servo1.initial_pos();
  servo2.initial_pos();
  servo3.initial_pos();

}

void loop() {
  cmd.listen();

  if(servo_select != last_servo){
    servo_select = constrain(servo_select,1,3);
    last_servo = servo_select;
    servos[(uint8_t)(last_servo-1)].pwm_range[0] = last_pwm_range[0];
    servos[(uint8_t)(last_servo-1)].pwm_range[1] = last_pwm_range[1];
    last_pos = last_pos-1;
  }

  if(pos != last_pos){
    last_pos = pos;
    servos[(uint8_t)(last_servo-1)].set_pos((int)last_pos);
    //servo2.set_pos((int)last_pos);
    //servo3.set_pos((int)last_pos);
  }

  if(pwm_range[0] != last_pwm_range[0] || pwm_range[1] != last_pwm_range[1]){
    last_pwm_range[0] = pwm_range[0];
    last_pwm_range[1] = pwm_range[1];
    servos[(uint8_t)(last_servo-1)].pwm_range[0] = last_pwm_range[0];
    servos[(uint8_t)(last_servo-1)].pwm_range[1] = last_pwm_range[1];
    last_pos = last_pos-1;
  }

  if(test && (millis()-last_time)>= 1000){
    switch(seq){
      case 0:{
        pos=0;
        seq++;
        break;
      }
      case 1:{
        pos = 90;
        seq++;
        break;
      }
      case 2:{
        pos = 180;
        seq++;
        break;
      }
      case 3:{
        pos = 90;
        seq = 0;
        test = !test;
        break;
      }
    }
    last_time = millis();
  }
}
#else

float x_inclination=0, y_inclination=0, z_height=90;
float last_x_incl = 0, last_y_incl = 0, last_z_height =90;

void list(){
  cmd.list();
}

void command_config(){
  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("x",&x_inclination);
  cmd.addCommand("y",&y_inclination);
  cmd.addCommand("z",&z_height);

  cmd.begin(115200);
}

void setup(){
  float fist_pos = robot_IK.get_IK_Theta(eslabon::A,last_z_height,0,0);

  servo1.begin(350,2000,0,180,fist_pos,false);
  servo2.begin(450,2010,0,180,fist_pos,false);
  servo3.begin(430,2010,0,180,fist_pos,false);

  command_config();

  servo1.initial_pos();
  servo2.initial_pos();
  servo3.initial_pos();

}

void loop(){
  cmd.listen();
  if((last_x_incl != x_inclination) || (last_y_incl != y_inclination) || (last_z_height != z_height)){
    last_x_incl = x_inclination;
    last_y_incl = y_inclination;
    last_z_height = z_height;
    servo1.set_pos(robot_IK.get_IK_Theta(eslabon::A,last_z_height,last_x_incl*(deg2rads),last_y_incl*(deg2rads))-20);
    servo2.set_pos(robot_IK.get_IK_Theta(eslabon::B,last_z_height,last_x_incl*(deg2rads),last_y_incl*(deg2rads))-20);
    servo3.set_pos(robot_IK.get_IK_Theta(eslabon::C,last_z_height,last_x_incl*(deg2rads),last_y_incl*(deg2rads))-20);
  }
}

#endif
