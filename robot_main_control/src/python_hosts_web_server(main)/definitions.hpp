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

void ADC_test();
void M1_backward();
void M1_forward();
void M1_stop();
void M2_backward();
void M2_forward();
void M2_stop();

//#######################################################################
//#######################################################################

extern WiFiClient client;
extern const char* ssid;
extern const char* password;
extern const uint16_t port;
extern const char* host;                                 //# FOR WIFI FUNCTION

extern String rec_Message;
extern char holder;
extern bool client_Flag;
//#######################################################################
//#######################################################################





//#######################################################################
//#######################################################################
extern Adafruit_MCP3008 adc1;                   //FOR THE LIGHT BAR
extern Adafruit_MCP3008 adc2;
//#######################################################################
//#######################################################################




//#######################################################################
//#######################################################################
                                                //FOR ENCODER




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
extern const unsigned int M2_IN_1_CHANNEL;                 // FOR MOTOR CONTROLS
extern const unsigned int M2_IN_2_CHANNEL;

extern const unsigned int M1_I_SENSE;
extern const unsigned int M2_I_SENSE;

extern const float M_I_COUNTS_TO_A;
extern const unsigned int PWM_VALUE; 
//#######################################################################
//#######################################################################




#endif