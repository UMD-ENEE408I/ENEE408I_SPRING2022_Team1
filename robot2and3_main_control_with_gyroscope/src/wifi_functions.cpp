#include "definitions.hpp"

void send_and_recieve_message_to_client(){
  rec_Message = "";
  holder = '\0';

  if(client_Flag == true){
    while(!client.connect(host, port)){
      Serial.println("client connecting");
    }
    
    if (client.connected()) {
        Serial.println("client connected");
        client.flush();
        delay(100);
        client.write("Begin");

        while(client.available() == 0){
          delay(500);
        }
        Serial.println("is available");
        delay(500);
        while(holder != '\n'){
          holder = client.read();
          //Serial.println("holder is" + holder);
          rec_Message += holder;
        
          //delay(200);
        }
        client.flush();
    }

  }
}