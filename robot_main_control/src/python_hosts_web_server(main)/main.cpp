#include "definitions.hpp"


//################################
WiFiClient client;   // Listen for incoming clients
String rec_Message = "";
char holder;                     // FOR WIFI FUNCTION
bool client_Flag = true;
const char* ssid = "ARRIS-93FA";
const char* password = "BSY89A602856";
const uint16_t port = 8000;
const char* host = "192.168.0.14";   
//################################



//################################
Adafruit_MCP3008 adc1;
Adafruit_MCP3008 adc2;




//################################













//##################################################################################################
void setup() {
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  delay(100);

  Serial.begin(115200);


  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(false,true);
  //WiFi.begin(ssid, password);
  //while (WiFi.status() != WL_CONNECTED) {
  //  delay(500);
  //  Serial.print(".");
  //}

  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());


  adc1.begin(ADC_1_CS);  
  adc2.begin(ADC_2_CS); 
 
}










//##################################################################################################

void loop(){
  Encoder enc1(M1_ENC_A, M1_ENC_B);
  Encoder enc2(M2_ENC_A, M2_ENC_B);



  while(true){
    
    //send_and_recieve_message_to_client();
    //Serial.println("scope check and FINAL -->> " + rec_Message);

    //ADC_test();

    //Encoder_Test(enc1, enc2); //ask levi why passing in like this screws with it
    //long enc1_value = enc1.read();
    //long enc2_value = enc2.read();
    //Serial.print(enc1_value);
    //Serial.print("\t");
    //Serial.print(enc2_value);
    //Serial.println();







    //delay(6000);
    //client_Flag = true;
  }
}