#include "definitions.hpp"



void ADC_test(){

  int adc1_buf[8];
  int adc2_buf[8];

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




void Encoder_Test(Encoder enc1, Encoder enc2){
    // Create the encoder objects after the motor has
    // stopped, else some sort exception is triggered


    long enc1_value = enc1.read();
    long enc2_value = enc2.read();
    Serial.print(enc1_value);
    Serial.print("\t");
    Serial.print(enc2_value);
    Serial.println();
    //delay(10); //If you add this delay it sometimes misses encoder ticks on enc2 even though interrupts are used....


}