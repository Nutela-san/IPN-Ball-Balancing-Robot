#ifndef SERVO_CONTROLLER
#define SERVO_CONTROLLER

#include <Arduino.h>


float map_f(float valor, float min_rango_original, float max_rango_original ,float min_nuevo_rango, float max_nuevo_rango){
  return (valor - min_rango_original) * (max_nuevo_rango - min_nuevo_rango) / (max_rango_original - min_rango_original) + min_nuevo_rango;
}

class ServoControl{
public:
  uint8_t id_pwm_chanel, pwm_bit_resolution, pwm_freq, pwm_pin;
  unsigned int pwm_range[2]; // [0]:min, [1]:max
  unsigned int max_pwm_value = pow(2,pwm_bit_resolution)-1;
  float periodo = 1000.0/((float)pwm_freq); 
  
  ServoControl(uint8_t PWMchanel , uint8_t PWM_pin, uint8_t PWM_bitResotution=16, uint8_t PWM_Frequency = 50)
  {
    this->pwm_pin = PWM_pin;
    id_pwm_chanel = PWMchanel;
    pwm_bit_resolution = PWM_bitResotution;
    pwm_freq = PWM_Frequency;
  }

  void begin(){
    periodo = 1000.0/((float)pwm_freq);
    pwm_range[0] = (int)map_f(1,0,periodo,0,max_pwm_value);
    pwm_range[1] = (int)map_f(2,0,periodo,0,max_pwm_value);

    pinMode(pwm_pin,OUTPUT);
    ledcSetup(id_pwm_chanel,pwm_freq,pwm_bit_resolution);
    ledcAttachPin(pwm_pin,id_pwm_chanel);
    this->set_pos(90);
  }

  uint16_t degrees2pwm(uint8_t degrees)
  {
    return ((uint16_t)map(degrees, 0, 180, pwm_range[0], pwm_range[1]));
  }

  void set_pos(uint8_t degrees)
  {
    ledcWrite(id_pwm_chanel,degrees2pwm(degrees));
  }

  void print_info_servo()
  {
    Serial.println("-INFO DE CONFIG DE SERVO-");
    Serial.print("  Direccion de servo en modulo PCA9685: ");
    Serial.println(id_pwm_chanel);
    Serial.print("  Rangos de PWM actuales: ");
    Serial.print(pwm_range[0]);
    Serial.print(" - ");
    Serial.println(pwm_range[1]);
  }
};

#endif