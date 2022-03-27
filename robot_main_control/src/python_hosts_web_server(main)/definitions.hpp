#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP
// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>

//function prototypes
void send_and_recieve_message_to_client();
void Encoder_Print(); 

void ADC_test();
void M1_backward();
void M1_forward();
void M1_stop();
void M2_backward();
void M2_forward();
void M2_stop();
void pid_v1_control();
void whl_1_2_vl_PID_calculation();
void motor_move();
//#######################################################################
//#######################################################################
extern WiFiClient client;
extern const char* ssid;
extern const char* password;
extern const uint16_t port;
extern const char* host;                                 //# FOR WIFI

extern String rec_Message;
extern char holder;
extern bool client_Flag;
//#######################################################################
//#######################################################################





//#######################################################################
//#######################################################################
extern Adafruit_MCP3008 adc1;                   //FOR THE LIGHT BAR
extern Adafruit_MCP3008 adc2;
extern int* adc1_buf; // could change syntax back
extern int adc2_buf[8];
//#######################################################################
//#######################################################################




//#######################################################################
//#######################################################################                   
extern long enc1_value;                           //FOR ENCODER
extern long enc2_value;
//#######################################################################
//#######################################################################



//#######################################################################
//#######################################################################
extern const unsigned int M1_IN_1;
extern const unsigned int M1_IN_2;
extern const unsigned int M2_IN_1;
extern const unsigned int M2_IN_2;

extern const unsigned int M1_IN_1_CHANNEL;
extern const unsigned int M1_IN_2_CHANNEL;
extern const unsigned int M2_IN_1_CHANNEL;                 // FOR MOTOR
extern const unsigned int M2_IN_2_CHANNEL;

extern const unsigned int M1_I_SENSE;
extern const unsigned int M2_I_SENSE;

extern const float M_I_COUNTS_TO_A;
extern unsigned int M1_PWM_VALUE; 
extern unsigned int M2_PWM_VALUE; 

//#######################################################################
//#######################################################################





//#######################################################################
//#######################################################################
extern unsigned long prev_twinky_time;
extern float twinky_one;
extern float twinky_two;
extern float twinky_one_speed;
extern float twinky_two_speed;

extern float whl1_vl_PID_error;
extern float whl2_vl_PID_error;

extern float whl1_vl_PID_P;
extern float whl2_vl_PID_P;

extern float whl1_vl_PID_I;
extern float whl2_vl_PID_I;                                  //FOR Motor PID Controller
                             
extern float whl1_vl_PID_D;
extern float whl2_vl_PID_D;

extern float whl1_vl_PID_KP;
extern float whl2_vl_PID_KP;

extern float whl1_vl_PID_KI;
extern float whl2_vl_PID_KI;

extern float whl1_vl_PID_KD;
extern float whl2_vl_PID_KD;

extern float whl1_vl_PID_error_prev;
extern float whl2_vl_PID_error_prev;

extern unsigned long whl1_vl_PID_D_time_prev;
extern unsigned long whl2_vl_PID_D_time_prev;

extern float whl1_vl_PID_out;
extern float whl2_vl_PID_out;

extern unsigned long current_time;

//#######################################################################
//#######################################################################














#endif