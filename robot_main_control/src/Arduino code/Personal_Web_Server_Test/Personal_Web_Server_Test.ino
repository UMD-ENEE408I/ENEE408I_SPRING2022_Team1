/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
//#include <Arduino.h>
#include <WiFi.h>
//#include <Wire.h>
//#include <SPI.h>

// Replace with your network credentials
const char* ssid = "ARRIS-93FA";
const char* password = "BSY89A602856";

// Set web server port number to 80
WiFiServer server(80);
WiFiClient client;
// Variable to store the HTTP request
String rec_Message = "";
char holder;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;
String currentLine = "";                // make a String to hold incoming data from the client

//##################################################################################################

void waitforclient(){
  while(!client.available()){
    delay(500);
    Serial.println("waiting for client response");
  }
}


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
  server.begin();
}

//##################################################################################################



void loop(){
  WiFiClient client = server.available();   // Listen for incoming clients
  if(client){
    Serial.println("client first time connected");
  }

    
  //if (client.connected()) {  
      client.write("Begin");
      delay(5000);
      if(client.available()){
        Serial.println("is available");
        while(holder != '\n'){
          Serial.println("INEHRE");
          holder = client.read();
          rec_Message += holder;
        }
        Serial.println("FINAL MESSAGE ->>" + rec_Message);
      }
 // }


    
}
