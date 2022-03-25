#include "definitions.hpp"

void send_and_recieve_message_to_client(WiFiClient client){
  rec_Message = "";
  holder = '\0';

  if(client_Flag == true){
    while(!client.connect(host, port)){
      Serial.println("client connecting");
    }
      
    if (client.connected()) {
        Serial.println("client connected");
        client.flush();
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
        client.flush();
    }

  }
}