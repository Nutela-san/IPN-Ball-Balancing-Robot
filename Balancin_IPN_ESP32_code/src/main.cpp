#include <Arduino.h>
#include <InterCom.h>
#include <ServoController.h>

SimpleCommand cmd;

ServoControl servo1(0,5);

float pos = 0, last_pos = 0;

void list(){
  cmd.list();
}

void setup() {
  servo1.begin();

  cmd.enable_echo(true);
  cmd.addCommand("list",list);
  cmd.addCommand("p",&pos);

  cmd.begin(115200);
}

void loop() {
  cmd.listen();

  if(pos != last_pos){
    last_pos = pos;
    servo1.set_pos((int)last_pos);
  }
}

