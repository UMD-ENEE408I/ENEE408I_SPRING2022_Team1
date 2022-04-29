#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP
// Load libraries
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

//function prototypes
void send_and_recieve_message_to_client();

void reset_variables();
void Encoder_Print(); 
void read_Light_bar();
void M1_backward();
void M1_forward();
void M1_stop();
void M2_backward();
void M2_forward();
void M2_stop();
void pid_v1_control();
void whl_1_2_vl_PID_calculation();
void motor_move();
void pid_lf_control();
void read_Light_bar2();
void pid_lf2_control();
void GYRO_PID_loop();

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
extern String move_to_make;
//extern int index;
//extern int prev_index;
//#######################################################################
//#######################################################################





//#######################################################################
//#######################################################################
extern Adafruit_MCP3008 adc1;                   //FOR THE LIGHT BAR
extern Adafruit_MCP3008 adc2;
extern int* adc1_buf; // could change syntax back
extern int adc2_buf[8];
extern int* adc_buf;
extern int* adc_buf2;

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
extern int M1_PWM_VALUE; 
extern int M2_PWM_VALUE; 
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

extern bool foward_Flag;
//#######################################################################
//#######################################################################




//#######################################################################
//#######################################################################
extern unsigned long prev_line_follow_time;    
extern int LightBar_Left_Sum;
extern int LightBar_Right_Sum;
extern float line_PID_error;
extern float kp1_divider;
extern float line_follow_PID_KP;
extern float line_follow_PID_KI;   
extern float line_follow_PID_KD;                               //FOR LINE FOLLOW PID LOOP 
extern float line_follow_PID_P;                                                            
extern float line_follow_PID_I; 
extern float line_follow_PID_D;
extern float line_PID_error_prev;
extern float line_follow_PID_out;
extern float twinky_max;
extern float twinky_min;

extern float average;
extern short b;
extern float adjustment;
extern float position;
extern float acc;
extern float line_follow_PID_KP2;
extern float line_follow_PID_KI2;
extern float line_follow_PID_KD2;
extern float kp2_divider;
//#######################################################################
//#######################################################################







//#######################################################################
//#######################################################################
extern long desired_enc1_value;
extern long desired_enc2_value;                                                       //FOR Intersection Logic
extern float dead_end_thresh;
extern float desired_degree_value;
extern bool right_most_flag;
extern bool middle_flag;
extern bool left_most_flag;
//#######################################################################
//#######################################################################




//#######################################################################
//#######################################################################
extern Adafruit_MPU6050 mpu;
extern unsigned long gyro_prev_time;
extern unsigned long gyro_current_time;                                                 
extern float gyro_degrees;
extern float gyro_PID_error;
extern float gyro_PID_error_prev;
extern float gyro_PID_P;
extern float gyro_PID_I;                                                            //FOR GYRO PID control
extern float gyro_PID_D;
extern float gyro_KP_divider;
extern float gyro_PID_KP;
extern float gyro_PID_KI;
extern float gyro_PID_KD;
extern float gyro_PID_out;

extern bool gyro_foward_flag;
//#######################################################################
//#######################################################################

#endif