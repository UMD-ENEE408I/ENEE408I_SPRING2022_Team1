#include <Adafruit_MCP3008.h>
#include <Encoder.h>

////////////////////////////////////BEGIN ENCODER INIT
const unsigned int M1_ENC_A = 6;
const unsigned int M1_ENC_B = 7;
const unsigned int M2_ENC_A = 8;
const unsigned int M2_ENC_B = 9;

Encoder enc1(M1_ENC_A, M1_ENC_B);
Encoder enc2(M2_ENC_A, M2_ENC_B);
////////////////////////////////////END ENCODER INIT

////////////////////////////////////BEGIN SENSOR INIT
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;

const unsigned int ADC_1_CS = A3;
const unsigned int ADC_2_CS = A2;

const unsigned int RF_CS = A4;
/////////////////////////////////////END SENSOR INIT

/////////////////////////////////////BEGIN MOTOR INIT
const unsigned int M1_IN_1 = 2;
const unsigned int M1_IN_2 = 3;
const unsigned int M2_IN_1 = 5;
const unsigned int M2_IN_2 = 4;

const unsigned int M1_I_SENSE = A1;
const unsigned int M2_I_SENSE = A0;

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;

const unsigned int PWM_VALUE = 35;

void M1_backward(int offset) {
  analogWrite(M1_IN_1, PWM_VALUE + offset);
  analogWrite(M1_IN_2, 0);
}

void M1_forward(int offset) {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, PWM_VALUE + offset);
}

void M1_stop() {
  analogWrite(M1_IN_1, 0);
  analogWrite(M1_IN_2, 0);
}

void M2_backward(int offset) {
  analogWrite(M2_IN_1, PWM_VALUE + offset);
  analogWrite(M2_IN_2, 0);
}

void M2_forward(int offset) {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, PWM_VALUE + offset);
}

void M2_stop() {
  analogWrite(M2_IN_1, 0);
  analogWrite(M2_IN_2, 0);
}

////////////////////////////////END MOTOR INIT

void setup() {
  // put your setup code here, to run once:
  //////////////BEGIN SENSOR SETUP
  Serial.begin(115200);      // Also Encoder setup line

  pinMode(RF_CS, OUTPUT);
  digitalWrite(RF_CS, HIGH); // Without this the nRF24 will write to the SPI bus
                             // while the ADC's are also talking

  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS);  
  //////////////END SENSOR SETUP

  //////////////BEGIN MOTOR SETUP
  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
  //////////////END MOTOR SETUP
}

void loop() {
  // put your main code here, to run repeatedly:
  int adc1_buf[8];
  int adc2_buf[8];
  int left = 0;
  int right = 0;
  int t_start = micros();
  for (int i = 0; i < 8; i++) {
    adc1_buf[i] = adc1.readADC(i);
    adc2_buf[i] = adc2.readADC(i);
  }
  int t_end = micros();

  for (int i = 0; i < 8; i++) {
    Serial.print(adc1_buf[i]); Serial.print("\t");
    Serial.print(adc2_buf[i]); Serial.print("\t");
  }
  Serial.print(t_end - t_start); Serial.print("\t");                   //TIME OF MEASUREMENT
  if(adc1_buf[0] < 650 || adc1_buf[1] < 650 || adc1_buf[2] < 650 || adc2_buf[0] < 650 || adc2_buf[1] < 650) {
    Serial.print("NEED TO GO R");
    right = 1;
  }
  if(adc1_buf[4] < 650 || adc2_buf[4] < 650 || adc1_buf[5] < 650 || adc2_buf[5] < 650 || adc1_buf[6] < 650) {
    Serial.print("NEED TO GO L");
    left = 1;
  }
  if(left == 1) {
    M2_forward(5);
  } else {
    M2_forward(0);
  }
  if(right == 1) {
    M1_forward(5);
  } else {
    M1_forward(0);
  }
  if(adc1_buf[3] > 650) {
    M1_stop();
    M2_stop();
    Serial.print("\t Stop");
  }
  Serial.println();
  int e1 = enc1.read(), e2 = enc2.read();
  Serial.print(e1);
  Serial.print("\t");
  Serial.print(e2);
  Serial.print("\t");
  Serial.print(e1 + e2);
  Serial.println();
  delay(100);
}
