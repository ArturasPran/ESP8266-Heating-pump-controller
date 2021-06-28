/* Arturas Pranskunas
 * This is the second part of "ESP8266 Heating pump controller & monitoring station"  project
 * The project can be found at https://ehelper.tk/esp8266-heating-pump-controller-monitoring-station-part-2/
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 */
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <ESP8266WiFi.h>
#include <espnow.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for SSD1306 display connected using software SPI (default case):
#define OLED_MOSI  12 //D6
#define OLED_CLK   14 //D5
#define OLED_DC    15 //D8
#define OLED_CS    0  //Does not exist
#define OLED_RESET 13 //D67
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);
//DHT
#define DHT11_pin 4 //Digital pin D2
#define DHTTYPE DHT11
DHT dht(DHT11_pin, DHTTYPE);
//Button
#define buttonPin 5 // D1

//Heater data
float supplytemp;
float returntemp;
float psupplytemp;
float preturntemp;
bool pumpState = true;

//Settings
const float roomSetpoint = 23;
const int cInterval = 60*1000;
const int tempInterval = 10*60*1000;
const int signalInterval = 10*60*1000;
const int lcdInterval = 8*1000;
const int srInterval = 5*6*1000;

//Variables
float roomtemp;
float proomtemp;
float humidity;
float phumidity;
bool lcdState = true;
unsigned long temppreviousMillis = 0;
unsigned long previousMillis = 0;
unsigned long lcdpreviousMillis = 0;
unsigned long srpreviousMillis = 0;
int displayPage;
bool infoPage = true;
bool error;
String infoText;
bool boolResponse;
bool responseReceived;
bool recirculation;
int  recirculationOffset;
//Button
int buttonState;
int lastButtonState = LOW;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers


//THE MAC Address of your receiver 
//uint8_t broadcastAddress[] = {0xF4, 0xCF, 0xA2, 0xF7, 0x4F, 0x10};
uint8_t broadcastAddress[] = {0xCC, 0x50, 0xE3, 0x7C, 0xD9, 0x7C};
// Variable to store if sending data was successful
String success;
//Data structure

//Receiving structure
typedef struct struct_inMessage {
  float supplytemp;
  float returntemp;
  bool pumpState;
  } struct_message;
//Structure to send
typedef struct struct_sendMessage {
  bool state;
  bool request;
  }struct_sendMessage;
  
  
//Structure to hold incoming readings
struct_message incomingReadings;
int incomingstatus;

// Callback when data is sent
void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("Last Packet Send Status: ");
  if (sendStatus == 0){
    Serial.println("Delivery success");
    boolResponse = false;
  }
  else{
    Serial.println("Delivery fail");
    boolResponse = true;
  errorMessage('R');
  }
}
// Callback when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) {
  memcpy(&incomingReadings, incomingData, sizeof(incomingReadings));
  Serial.print("Bytes received: ");
  Serial.println(len);
  supplytemp = incomingReadings.supplytemp;
  returntemp = incomingReadings.returntemp;
  pumpState = incomingReadings.pumpState;
  Serial.println("S-"+String(incomingReadings.supplytemp)+ " R-"+String(incomingReadings.returntemp)+ " Pump-"+String(incomingReadings.pumpState));
  responseReceived = true;
}

void setup() {
    Serial.begin(9600);
    // Set device as a Wi-Fi Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    
    // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
    if(!display.begin(SSD1306_SWITCHCAPVCC)) {
      Serial.println(F("SSD1306 allocation failed"));
      for(;;); // Don't proceed, loop forever
  }
    //Button
    pinMode(buttonPin,INPUT);
    
    //DHT sensor setup
    pinMode(DHT11_pin, INPUT);
    dht.begin();
    
  // Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  drawLoading();
    // Set ESP-NOW Role
  esp_now_set_self_role(ESP_NOW_ROLE_COMBO);

  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_COMBO, 1, NULL, 0);
  
  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(OnDataRecv);

  sampleTempData(0);
  errorMessage('O');

}

void loop() {
  // put your main code here, to run repeatedly:
  mainProg();
  lcdButton(); 

}
int deltaSimbol(float current, float previous){
  if (current - previous > 0.05){
    return 24; //↑
    }
  else if(current - previous < -0.05){
    return 25; // ↓
    }
    else{
      return 246; // ≈
      }  
  }
void lcdButton(){
  int reading = digitalRead(buttonPin);

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        lcdState = !lcdState;
      }
    }
  }
  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
  
  }  
void drawLoading(){
  display.clearDisplay();
  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,25);             // Start at top-left corner
  display.println(F("Kraunasi.."));    //ENG: Loading...
  display.display();
  }  
void drawRoomData(){
  display.clearDisplay();
  
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("KAMBARIO TEMPERATURA:")); //ENG: Room temp.

  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,10);
  //display.cp437(true);
  display.write(deltaSimbol(roomtemp,proomtemp));
  display.setCursor(18,10);             // Start at top-left corner
  display.println((roomtemp != 0) ? String(roomtemp) + "C" : "-- C");

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,34);             // Start at top-left corner
  display.println(F("SANTYKINIS DREGNUMAS:")); //ENG: Relative humidity

  display.setTextSize(3);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,44);
  //display.cp437(true);
  display.write(deltaSimbol(humidity,phumidity));
  display.setCursor(18,44);             // Start at top-left corner
  display.println((humidity != 0) ? String(humidity) + "%" : "-- %");
  
  display.display();
  }
void drawPumpData(){
  display.clearDisplay();
  
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("TIEKIMO TEMPERATURA:")); //ENG: Supply temp

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,10);
  //display.cp437(true);
  display.write(deltaSimbol(supplytemp,psupplytemp));
  display.setCursor(17,10);             // Start at top-left corner
  display.println((supplytemp != 0) ? String(supplytemp) + "C" : "-- C");

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,28);             // Start at top-left corner
  display.println(F("GRIZTAMO TEMPERATURA:")); //ENG: Return temp.

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,38);
  //display.cp437(true);
  display.write(deltaSimbol(returntemp,preturntemp));
  display.setCursor(17,38);             // Start at top-left corner
  display.println((returntemp != 0) ? String(returntemp)+ "C" : "-- C");

  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,56);             // Start at top-left corner
  display.println(F("POMPOS BUSENA:")); //ENG: Pump state

  display.setTextSize(2);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(90,50);             // Start at top-left corner
  display.println((pumpState == true) ? "ON" : "OFF");
  
  display.display();
  }
void drawInfoTab(){
  display.clearDisplay();
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,0);             // Start at top-left corner
  display.println(F("INFORMACIJA:")); //ENG: Info
  display.setTextSize(1);             // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE);        // Draw white text
  display.setCursor(0,10);
  display.println(infoText);
  display.display();
  
  }  
void sampleTempData(unsigned long currentMillis){
    float temp = dht.readTemperature();
    float hum = dht.readHumidity();
    Serial.println(temp);
    Serial.println(hum);

  if (isnan(temp) || isnan(hum)) {
        Serial.println(F("Failed to read from DHT sensor!"));
        errorMessage('D');
        return;
  }
  else{
    if(currentMillis - temppreviousMillis >= tempInterval) {
        proomtemp = roomtemp;
        phumidity = humidity;
        temppreviousMillis = currentMillis;
      }
        roomtemp = temp -0.9; // Sensor correction factor -0.4
        humidity = hum +10; //Sensor correction factor
    }
    
  }  
void mainProg(){
  unsigned long currentMillis = millis();
  //Send request
  if(currentMillis - previousMillis >= cInterval - recirculationOffset){
    //Set error to false on each iteration to check wether it persists.
    error = false;
    errorMessage('O');
    requestData();
    //Read room temp
    sampleTempData(currentMillis);
    previousMillis = currentMillis; 
    }
  //Perform pump controll if response is received
  // Main heating control check every 3 min. pump circulate ~18liters in 3 min.   
  if(responseReceived == true){
        heatingControl();
        responseReceived = false;
        //Update previous heater temperature readings with greater delay.
        if(currentMillis - srpreviousMillis >= srInterval){
          psupplytemp = supplytemp;
          preturntemp = preturntemp;
          srpreviousMillis = currentMillis;
          }
        }
  if(lcdState == true){
      //Screen update  
      if(currentMillis - lcdpreviousMillis >= lcdInterval){
        if(displayPage == 0){
          drawRoomData();
          displayPage++;
          }
        else if (displayPage == 1){
          drawPumpData();
          displayPage++;
          }
        else if (displayPage == 2 && infoPage == true){
          drawInfoTab();
          displayPage++;
          }  
        else {
          displayPage = 0;
          }
          lcdpreviousMillis = currentMillis;    
    }
    
    }
    else{
      display.clearDisplay();
      display.display();
      }
  }
void requestData(){
  struct_sendMessage sMessage;
  sMessage.request = true;
  esp_now_send(broadcastAddress, (uint8_t * ) &sMessage, sizeof(sMessage));
  Serial.println("Request data: " + String(sMessage.request));
  }
    
bool pumpControl(bool state){
  struct_sendMessage sMessage;

  if(state == true && pumpState == false){
    sMessage.state = true;
    pumpState = true;
    sMessage.request = false;
    esp_now_send(broadcastAddress, (uint8_t * ) &sMessage, sizeof(sMessage));  
    }
  else if (state == false && pumpState == true){
    sMessage.state = false;
    pumpState = false;
    sMessage.request = false;
    esp_now_send(broadcastAddress, (uint8_t * ) &sMessage, sizeof(sMessage));  
    }

  }  
  
void heatingControl(){
  bool localPumpState = true;
  
  if(supplytemp == -127 || returntemp == -127){
    Serial.println("Failed to retrieve data from heater sensors. Starting alternative heating control.");
    errorMessage('S');
  //Alternative heating control if one of temperature sensors fails to return valid reading.
  if(roomtemp < roomSetpoint + 2.5){
    pumpControl(true);
  }
  else{
    pumpControl(false);
  }
    return;
    }
   // SafeMode
  if(supplytemp > 80){
    pumpControl(true);
    Serial.println("High temperature is reached. Keeping pump ON");
    errorMessage('H');
    return;
    }
    //recirculationOffset = 0;
    //Regular Control
  if(roomtemp < roomSetpoint && supplytemp > 48 && (supplytemp - returntemp) > 10 && returntemp < 48){ 
    localPumpState = true;
    Serial.println("Room setpoint is not reached. Turning pump ON");
    }
  else if (supplytemp < 22 && returntemp <22){
    localPumpState = true;
    recirculation = false;
    Serial.println("Pump is off on thermostat. No more fuel is left");  
    }
  else if (roomtemp >= roomSetpoint){
    localPumpState = false;
    recirculation = true;
    Serial.println("Room set point is reached. Pump is OFF");
    }
  else if(supplytemp - returntemp < 10){
    Serial.println("Pump is OFF until return temperature drops");
    localPumpState = false;
    }
  else if (returntemp > 48){
    Serial.print("Return temperature above 48C. Turning pump off");
    localPumpState = false;
    }  
    else{
      Serial.println("Supply temp does not meet minimum T for pump to circulate. Pump is on standby");
      localPumpState = true;
      }
  //Recirculation    
  if (supplytemp > 48 && returntemp < 32 && recirculation == true){
    Serial.println("Recirculation is active.");
    localPumpState = true;
    recirculationOffset = 0; // minus from next check, it can't exceed cInterval
    }    
    pumpControl(localPumpState);
  }
void errorMessage(char errorname){
  String dht = "- Kambario sensoriaus klaida!\n"; //ENG: Room sensor faiure
  String dsSensor = "- Pecio sensoriaus gedimas!\n"; //ENG: Controller sensor failure
  String hightemp = "- Auksta temperatura katile!\n"; //ENG: High wood burner temperature
  String remoteResponse = "- Nera signalo su pompos valdikliu!\n"; //ENG: No signal with pump controller

  switch(errorname){
    case 'D':
    if(error == false){
      infoText = dht;
      error = true;
      }
      else{
        infoText += dht;
        }
      break;
    case 'S':
      if(error == false){
      infoText = dsSensor;
      error = true;
      }
      else{
        infoText += dsSensor;
        }
      break;
    case 'H':
      if(error == false){
      infoText = hightemp;
      error = true;
      }
      else{
        infoText += hightemp;
        }
    case 'R':
    if(boolResponse == true){
      if(error == false){
        infoText = remoteResponse;
        error = true;
      }
      else{
        infoText += remoteResponse;
        }
      }    
      break;   
    default:
    infoText = "- SISTEMA VEIKIA NORMALIU REZIMU.\n";  //ENG: No issues detected. System functioning normally.
    }
  }  
