#ifndef SERVO_CONTROLLER
#define SERVO_CONTROLLER

#include <Arduino.h>


float map_f(float valor, float min_rango_original, float max_rango_original ,float min_nuevo_rango, float max_nuevo_rango){
  return (valor - min_rango_original) * (max_nuevo_rango - min_nuevo_rango) / (max_rango_original - min_rango_original) + min_nuevo_rango;
}

class ServoControl{
public:
  uint8_t id_pwm_chanel, pwm_bit_resolution, pwm_freq, pwm_pin;
  float pwm_range[2]; // [0]:min, [1]:max
  unsigned int max_pwm_value;
  float periodo; 

  uint8_t move_range[2];     // [0]:min, [1]:max
  uint8_t curret_pos;
  bool complementar = false; // variable para activar el complemento de los angulos
  uint8_t init_pos;
  
  ServoControl(uint8_t PWMchanel , uint8_t PWM_pin, uint8_t PWM_bitResotution=12, uint8_t PWM_Frequency = 50)
  {
    this->pwm_pin = PWM_pin;
    id_pwm_chanel = PWMchanel;
    pwm_bit_resolution = PWM_bitResotution;
    pwm_freq = PWM_Frequency;
  }

  void begin(){
    max_pwm_value = pow(2,pwm_bit_resolution)-1;
    periodo = 1000.0/((float)pwm_freq);
    pwm_range[0] = map_f(1.0,0.0,periodo,0.0,max_pwm_value);
    pwm_range[1] = map_f(2.0,0.0,periodo,0.0,max_pwm_value);
    move_range[0] = 0;
    move_range[1] = 180;


    pinMode(pwm_pin,OUTPUT);
    ledcSetup(id_pwm_chanel,pwm_freq,pwm_bit_resolution);
    ledcAttachPin(pwm_pin,id_pwm_chanel);
    init_pos = 90;
  }

  void begin(unsigned int pwm_valor_min, unsigned int pwm_valor_max, uint8_t rango_movimiento_min, uint8_t rango_movimiento_max, uint8_t pos_inicial, bool invertir){

    pwm_range[0] = pwm_valor_min;
    pwm_range[1] = pwm_valor_max;
    move_range[0] = rango_movimiento_min;
    move_range[1] = rango_movimiento_max;
    curret_pos = pos_inicial;
    init_pos = pos_inicial;
    complementar = invertir;

    pinMode(pwm_pin,OUTPUT);
    ledcSetup(id_pwm_chanel,pwm_freq,pwm_bit_resolution);
    ledcAttachPin(pwm_pin,id_pwm_chanel);
    this->set_pos(0);
  }

  uint16_t degrees2pwm(uint8_t degrees)
  {
    if (complementar)
    {
      degrees = (uint8_t)180 - degrees;
    }
    return ((uint16_t)map(degrees, 0, 180, pwm_range[0], pwm_range[1]));
  }

  void set_pos(uint8_t degrees)
  {
    if (degrees >= move_range[0] && degrees <= move_range[1])
    {
      ledcWrite(id_pwm_chanel,degrees2pwm(degrees));
      curret_pos = degrees;
    }
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

  void initial_pos()
  {
    set_pos(init_pos);
  }
};

#endif