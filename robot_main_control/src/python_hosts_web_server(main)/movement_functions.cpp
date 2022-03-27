#include "definitions.hpp"





void Encoder_Print(){
  Serial.print(enc1_value);
  Serial.print("\t");
  Serial.print(enc2_value);
  Serial.println();
}

void ADC_test(){

  //int adc1_buf[8]; // could make these extern, update: I did
  //int adc2_buf[8];

  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); 
    Serial.print("\t");
    Serial.print(adc2_buf[i]); 
    Serial.print("\t");
  }

  Serial.print(t_end - t_start);
  Serial.println();

  delay(100);



}





void M1_backward() {
  ledcWrite(M1_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M1_forward() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, PWM_VALUE);
}

void M1_stop() {
  ledcWrite(M1_IN_1_CHANNEL, 0);
  ledcWrite(M1_IN_2_CHANNEL, 0);
}

void M2_backward() {
  ledcWrite(M2_IN_1_CHANNEL, PWM_VALUE);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}

void M2_forward() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, PWM_VALUE);
}

void M2_stop() {
  ledcWrite(M2_IN_1_CHANNEL, 0);
  ledcWrite(M2_IN_2_CHANNEL, 0);
}





void pid_v1_control(){
  twinky_one = twinky_one + (millis() - prev_twinky_time)*twinky_one_speed; // multiply by some constant to keep pushing up the twinky_one distance
  twinky_two = twinky_two + (millis() - prev_twinky_time)*twinky_two_speed; // multiply by some constant to keep pushing up the twinky_one distance 

  // COMPUTE PID VL OUTPUT
  whl_1_2_vl_PID_calculation();

}


void whl_1_2_vl_PID_calculation(){
  // ERROR
  whl1_vl_PID_error = twinky_one - enc1_value; //this should be the desired - the current_read, twinky_one has been pushed up to a desired position.
  whl2_vl_PID_error = twinky_two - enc2_value;

  // PROPORTIONAL
  whl1_vl_PID_P = whl1_vl_PID_error * whl1_vl_PID_KP ;



}

















































