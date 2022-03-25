#ifndef DEFINITIONS_HPP
#define DEFINITIONS_HPP
// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>
//#include <Encoder.h>
#include <Adafruit_MCP3008.h>






// Replace with your network credentials
const char* ssid = "ARRIS-93FA";
const char* password = "BSY89A602856";
const uint16_t port = 8000;
const char* host = "192.168.0.14";

// Set web server port number to 80

// Variable to store the HTTP request
extern String rec_Message;
extern char holder;
extern bool client_Flag;


// Define timeout time in milliseconds (example: 2000ms = 2s)
extern String currentLine;                // make a String to hold incoming data from the client












#endif