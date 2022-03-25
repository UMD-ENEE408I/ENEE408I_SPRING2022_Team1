/*********
  Rui Santos
  Complete project details at https://randomnerdtutorials.com  
*********/
#include "definitions.hpp"

// Variable to store the HTTP request
String rec_Message = "";
char holder;
bool client_Flag = true;


String currentLine = "";     // make a String to hold incoming data from the client



//##################################################################################################
void setup() {
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  delay(100);

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




void send_and_recieve_message_to_client(WiFiClient client){
  rec_Message = "";
  holder = '\0';

  if(client_Flag == true){
    while(!client.connect(host, port)){
      Serial.println("client connecting");
    }
      
    if (client.connected()) {
        Serial.println("client connected");
        client.write("Begin");

        while(client.available() == 0){
          delay(500);
        }
        Serial.println("is available");

        while(holder != '\n'){
          holder = client.read();
          Serial.println("holder is" + holder);
          rec_Message += holder;
        //Serial.println("FINAL MESSAGE ->> " + rec_Message);
        delay(200);
        }
    }

  }
}



void loop(){
  WiFiClient client;   // Listen for incoming clients
  send_and_recieve_message_to_client(client);
  Serial.println("scope check and FINAL -->> " + rec_Message);
  

  delay(6000);  
  
  //client_Flag = true;
}