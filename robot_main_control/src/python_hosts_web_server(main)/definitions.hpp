#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP
// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
#include <Encoder.h>
#include <Adafruit_MCP3008.h>


void send_and_recieve_message_to_client(WiFiClient client);
void ADC_test();
void Encoder_Test(Encoder, Encoder);
//#######################################################################
//#######################################################################
// Replace with your network credentials

//extern WiFiClient client;
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
extern Adafruit_MCP3008 adc1;
extern Adafruit_MCP3008 adc2;
const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;




const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;





#endif