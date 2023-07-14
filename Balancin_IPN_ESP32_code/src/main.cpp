#include <Arduino.h>
#include <InterCom.h>
#include <ServoController.h>

uint8_t pines_servos[3]= {13,26,14};

SimpleCommand cmd;

ServoControl servo1(0,pines_servos[0],14,50), servo2(1,pines_servos[1],14,50), servo3(2,pines_servos[2],14,50);


float pos = 20, last_pos = 30;
bool test = false;

unsigned long last_time = 0;

void list(){
  cmd.list();
}

void popo(){
  servo1.print_info_servo();
}

void doTest(){
  last_time = millis();
  test = !test;
}

void setup() {
  servo1.begin(400,2000,0,120,30,true);
  servo2.begin(450,2000,0,120,30,true);
  servo3.begin(400,2000,0,120,30,true);

  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("test",doTest);
  cmd.addCommand("min",&servo1.pwm_range[0]);
  cmd.addCommand("max",&servo1.pwm_range[1]);
  cmd.addCommand("p",&pos);
  

  cmd.begin(115200);


  servo1.initial_pos();
  servo2.initial_pos();
  servo3.initial_pos();

}

void loop() {
  cmd.listen();

  if(pos != last_pos){
    last_pos = pos;
    servo1.set_pos((int)last_pos);
    servo2.set_pos((int)last_pos);
    servo3.set_pos((int)last_pos);
  }

  if(test && (millis()-last_time)>= 1500){
    if(pos < 30){
      pos = 80;
    }
    else if(pos > 70){
      pos = 20;
    }
    last_time = millis();
  }
}

