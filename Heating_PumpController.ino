/* Arturas Pranskunas
 * This is the first part of "ESP8266 Heating pump controller & monitoring station" project
 * The project can be found at https://ehelper.tk/esp8266-heating-pump-controller-monitoring-station-part-1/
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include <espnow.h>
#include <ESP8266WiFi.h>
/*
 * Dallas sensor 18b20
 */
#include <OneWire.h>
#include <DallasTemperature.h>

const int relPin = 0;
const int bled = 1; //tx
const int eled = 3; //rx
//Sensors
const int oneWireBus = 2;
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

//Variables
bool relState = true; // Default ON
bool sensorError;
bool statusLed;
bool sampleTemp;
unsigned long previousErrorMillis = 0;
unsigned long previousStatusMillis = 0;
int eLedState = HIGH;
int bLedState = HIGH;
int statusLedCounter;


//THE MAC Address of designated receiver 
uint8_t broadcastAddress[] = {0xF4, 0xCF, 0xA2, 0xF7, 0x4F, 0x10};

typedef struct struct_inMessage {
  bool state;
  bool request;
  } struct_message;

typedef struct struct_outMessage {
  float supplytemp;
  float returntemp;
  bool pumpState;
  } struct_outMessage;
   
//Structure to hold incoming readings
struct_message incomingReadings;

void setup() {
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // relay ON by default
    pinMode(relPin, OUTPUT);
    digitalWrite(relPin, HIGH); //ON
    
//  GPIO TX activation as normal gpio pins
  pinMode(bled,FUNCTION_3);
  pinMode(bled, OUTPUT);
  digitalWrite(bled, HIGH);
//
  //GPIO RX error pin
  pinMode(eled,FUNCTION_3);
  pinMode(eled, OUTPUT);
  digitalWrite(eled, HIGH);
  delay(1000);
    //Sensors
    sensors.begin();

    // Init ESP-NOW
  if (esp_now_init() != 0) {
    return;
  }
   // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);  

}

void loop() {
  unsigned long currentMillis = millis();
  //Start blink red led if one of the sensors is not working
  if(sensorError == true){
    if(currentMillis - previousErrorMillis >= 1000){
      previousErrorMillis = currentMillis;
      
      if(eLedState == LOW){
        eLedState = HIGH;
        }
       else{
        eLedState = LOW;  
          }
      }
      digitalWrite(eled, eLedState);
    }
    if(statusLed == true){
      if(currentMillis - previousStatusMillis >= 1000){
      previousStatusMillis= currentMillis;
  
      if(bLedState == HIGH){
        bLedState = LOW;
        }
       else{
        bLedState = HIGH;
        statusLed = false;  
          }
      digitalWrite(bled,bLedState);
      }
    }

    if(sampleTemp == true){
      sensors.requestTemperatures();
      float tsupply = sensors.getTempCByIndex(0);
      float treturn = sensors.getTempCByIndex(1);
      if(tsupply == -127 || treturn == -127){
        sensorError = true;
        }
      struct_outMessage sMessage;
      sMessage.supplytemp = tsupply;
      sMessage.returntemp = treturn;
      sMessage.pumpState = relState;
      //Send message
      esp_now_send(broadcastAddress, (uint8_t * ) &sMessage, sizeof(sMessage));
      sampleTemp = false;
      }

}

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {

  if (sendStatus == 0){
//Do nothing
  }
  else{
//Do nothing
  }
}
// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  pumpControl(incomingReadings.state, incomingReadings.request);
 
}
void pumpControl(bool state, bool req ){
  //blink status led when message arrives
  if(statusLed == false){
    statusLed = true;
    }
  if(req == true){
    sampleTemp = true;
    return;
    }
    else{
      relayControl(state);
      }
  }
void relayControl(bool state){
  if(state == true && relState == false){ // ON
    digitalWrite(relPin, HIGH);
    relState = true;
    }
  else if(state == false && relState == true){ // OFF
    digitalWrite(relPin, LOW);
    relState = false;
    }
  }
