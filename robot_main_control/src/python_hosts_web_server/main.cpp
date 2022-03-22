/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <SPI.h>

// Replace with your network credentials
const char* ssid = "ARRIS-93FA";
const char* password = "BSY89A602856";
const uint16_t port = 8000;
const char * host = "192.168.0.14";

// Set web server port number to 80

// Variable to store the HTTP request
String rec_Message = "";
char holder;
bool client_Flag = true;


// Define timeout time in milliseconds (example: 2000ms = 2s)
String currentLine = "";                // make a String to hold incoming data from the client




//##################################################################################################
void setup() {
  Serial.begin(115200);
  // Initialize the output variables as outputs


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

//##################################################################################################



void loop(){
  WiFiClient client;   // Listen for incoming clients

  if(client_Flag == true){

    while(!client.connect(host, port)){
      Serial.println("client connecting");
    }
      
    if (client.connected()) {
        Serial.println("client connected");
        client.write("Begin");
        delay(7000);
        if(client.available()){
          Serial.println("is available");
          while(holder != '\n'){
            holder = client.read();
            Serial.println("holder is" + holder);
            rec_Message += holder;
          }
          Serial.println("FINAL MESSAGE ->>" + rec_Message);
          rec_Message = "";
          holder = '\0';
        }
    }
  }


  delay(10000);
  //client_Flag = true;
}