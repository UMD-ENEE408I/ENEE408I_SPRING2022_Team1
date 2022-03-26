#include "definitions.hpp"


//################################
// Replace with your network credentials

WiFiClient client;   
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
Adafruit_MCP3008 adc2;            //FOR LIGHT BAR
const unsigned int ADC_1_CS = 2;
const unsigned int ADC_2_CS = 17;
//################################



//################################
const unsigned int M1_ENC_A = 39;
const unsigned int M1_ENC_B = 38;   // FOR ENCODER
const unsigned int M2_ENC_A = 37;
const unsigned int M2_ENC_B = 36;
//################################


//################################
const unsigned int M1_IN_1 = 13;
const unsigned int M1_IN_2 = 12;
const unsigned int M2_IN_1 = 25;
const unsigned int M2_IN_2 = 14;              // FOR MOTOR CONTROLS

const unsigned int M1_IN_1_CHANNEL = 1;
const unsigned int M1_IN_2_CHANNEL = 2;
const unsigned int M2_IN_1_CHANNEL = 3;
const unsigned int M2_IN_2_CHANNEL = 4;

const unsigned int M1_I_SENSE = 35;
const unsigned int M2_I_SENSE = 34;

const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8;  // we can write PWM wave with max of 2^8 - 1 = 255

const float M_I_COUNTS_TO_A = (3.3 / 1024.0) / 0.120;
const unsigned int PWM_VALUE = 90; 
//################################






//##################################################################################################
void setup() {
  pinMode(14,OUTPUT);
  digitalWrite(14,LOW);
  delay(2000);

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
 

  pinMode(M1_IN_1, OUTPUT);
  pinMode(M1_IN_2, OUTPUT);
  pinMode(M2_IN_1, OUTPUT);
  pinMode(M2_IN_2, OUTPUT);
  
  ledcAttachPin(M1_IN_1, M1_IN_1_CHANNEL);
  ledcAttachPin(M1_IN_2, M1_IN_2_CHANNEL);
  ledcAttachPin(M2_IN_1, M2_IN_1_CHANNEL);
  ledcAttachPin(M2_IN_2, M2_IN_2_CHANNEL);

  ledcSetup(M1_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M1_IN_2_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_1_CHANNEL, freq, resolution);
  ledcSetup(M2_IN_2_CHANNEL, freq, resolution);

  pinMode(M1_I_SENSE, INPUT);
  pinMode(M2_I_SENSE, INPUT);
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
    long enc1_value = enc1.read();
    long enc2_value = enc2.read();
    Serial.print(enc1_value);
    Serial.print("\t");
    Serial.print(enc2_value);
    Serial.println();


    //M1_forward();
    //M2_forward();
   // delay(1000);
    //M1_stop();
   // M2_stop();
   // delay(1000);
    //M1_backward();
   // M2_backward();
   // delay(1000);
   // M1_stop();
   // M2_stop();
   // delay(1000);


    //delay(6000);
    //client_Flag = true;
  }
}