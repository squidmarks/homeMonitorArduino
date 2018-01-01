#include <ArduinoJson.h>
#include <SocketIoClient.h>
#include <NTPClient.h>
#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>
#include <EEPROM.h>
SocketIoClient webSocket;

StaticJsonBuffer<200> jsonBuffer;
JsonObject& root = jsonBuffer.createObject();

char userID[25]="5a47c3db34d1ff35883b2962";
char machineID[25]="FFFFQ";
char websocketServerURL[25]="192.168.1.108";
const char* FWVer="1.00";
const char* ssid = "GerhardtWifi";
const char* password = "lovelivy";

const char* mqttServer = "m14.cloudmqtt.com";
const int mqttPort = 18545;
const char* mqttUser = "bkwujams";
const char* mqttPassword = "y7vbc9I_FNzm";
const char* topic = "houseData";
char message_buff[100];
#define interruptPin 0
#define reportTime 5000 //Time in millisec between MQTT reports
#define nozzleRating 1.0515 //milliter/s oil flow at 100psi
float oilPressure=0;
float lastOilPressure=100;
long volumeInmL;
float cycleVolume;
bool  toggle = false;
long lastMillis;
float flowRate=0;
int  volumeEEAddr=20;
long lastReconnectAttempt=0;
float avgOilPressure=0;
long numOilPresSamples=0;
boolean pumpOn=false;
long hourFlowVolume=0;
float hourFlowRate=0;
long lastOilFlowMeasTime;

WiFiClient wifiClient;
PubSubClient client(wifiClient);
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

void callback(char* topic, byte* payload, unsigned int length) {
  int i = 0;
  Serial.print("MQTT message recd: Topic: ");
  Serial.print(topic);
  Serial.print(", payload: ");
  // create character buffer with ending null terminator (string)
  for(i=0; i<length; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  String msgString = String(message_buff);
  Serial.println("Payload: " + msgString);
  if (msgString.equals("RESET")){
      volumeInmL=0;
      EEPROM.begin(512);
      EEPROM.put(volumeEEAddr,volumeInmL);
      EEPROM.commit();
      EEPROM.end();
      sendMQTTMessage();
    }
  Serial.println();
}

String macToStr(const uint8_t* mac)
{
  String result;
  for (int i = 0; i < 6; ++i) {
    result += String(mac[i], 16);
    if (i < 5)
      result += ':';
  }
  return result;
}

void setupOTA(){
  Serial.print("Setting up OTA...");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
  Serial.println("ok.");
}

void setupInterrupts(){
  noInterrupts();
  timer0_isr_init();
  timer0_attachInterrupt(timer0_ISR);
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec

  pinMode(interruptPin, INPUT);
  pinMode(BUILTIN_LED, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), flowSensorInterrupt, FALLING);

  interrupts();
}

void getWeather(){
  Serial.println("Getting weather...");
  HTTPClient http;  //Declare an object of class HTTPClient
  
  http.begin("https://api.openweathermap.org/data/2.5/weather?zip=01520,us&APPID=b89806224be88e2a850ad3185d86684d");  //Specify request destination
  int httpCode = http.GET();                                                                  //Send the request
  
  if (httpCode > 0) { //Check the returning code
  
    String payload = http.getString();   //Get the request response payload
    Serial.println(payload);                     //Print the response payload
  
  } else {
    Serial.println("Error getting weather:"+httpCode);
    String payload = http.getString();   //Get the request response payload
    Serial.println(payload);
  }
  http.end();   //Close connection  
}

void sendSocketData(){
  root["cycleVolume"] = cycleVolume;
  root["oilVolume"] = volumeInmL;
  if (hourFlowRate>0){   
    root["hourFlowRate"] = hourFlowRate;
    hourFlowRate=0;
  }
  //root["time"] = timeClient.getEpochTime()*1000;
  root["time"] = timeClient.getEpochTime();
  root["id"] = userID;
  
  String JSONMsg;
  root.printTo(JSONMsg);

  webSocket.emit("oilUsageData", (char*) JSONMsg.c_str());
}

void sendMQTTMessage(){
  //getWeather();
  String payload = "{\"cycleVolume\":";
  payload += round(cycleVolume);
  payload += ",\"oilVolume\":";
  payload += volumeInmL;
  if (hourFlowRate>0){   
    payload += ",\"hourFlowRate\":";
    payload += hourFlowRate;
    hourFlowRate=0;
  }
  payload += ",\"time\":";
  payload += timeClient.getEpochTime()*1000;
  payload += ",\"userID\":";
  payload += userID;
  payload += "\"}";  
  
  Serial.println("");
  if (client.connected()){
    Serial.print("Sending MQTT message: ");
    String topic = "homeMonitor/";
    topic += userID;
    topic="homeMonitor/oilUsageData";
    Serial.print(topic);
    Serial.print(payload);
//    if (client.publish((char*) topic.c_str(), (char*) payload.c_str()))
    if (client.publish("homeMonitor/oilUsageData", (char*) payload.c_str()))
      Serial.println("...sent");
        else Serial.println("...failed");
  } else Serial.println("Not connected to MQTT server");
}

void timer0_ISR (void) {
  int sensorValue = analogRead(A0);
  // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3.2V):
  float voltage = sensorValue * (3.2 / 1023.0);
  oilPressure=(voltage-0.5)*37.5;
  //oilFlow = 0.0289*oilPressure + 5.3601

  if (((oilPressure-lastOilPressure)>50)&&(!pumpOn)) {
    //Oil pump turned on
    cycleVolume=0;
    avgOilPressure=oilPressure;
    numOilPresSamples=1;
    pumpOn=true;
    lastOilPressure=oilPressure;
    lastMillis = millis();
    digitalWrite(BUILTIN_LED, LOW);
  }
  
  if (((lastOilPressure-oilPressure)>50)&&(pumpOn)) {
    //Oil pump turned off
    avgOilPressure=avgOilPressure/numOilPresSamples;
    cycleVolume=((avgOilPressure/100)*nozzleRating)*(millis()-lastMillis)/1000;
    volumeInmL+= round(cycleVolume);
    if ((millis()-lastOilFlowMeasTime)>3600000){
      //calc flow in mL/min
      hourFlowRate=hourFlowVolume/((millis()-lastOilFlowMeasTime)/60000);
      hourFlowVolume=0;
      lastOilFlowMeasTime=millis();
    } else hourFlowVolume+=round(cycleVolume);
    //sendMQTTMessage();
    sendSocketData();
    EEPROM.begin(512);
    EEPROM.put(volumeEEAddr,volumeInmL);
    EEPROM.commit();
    EEPROM.end();
    digitalWrite(BUILTIN_LED, HIGH);
  }

  avgOilPressure=avgOilPressure+oilPressure;
  ++numOilPresSamples;

  lastOilPressure=oilPressure;
  timer0_write(ESP.getCycleCount() + 80000000L); // 80MHz == 1sec
}

void socketIOConnect(const char * payload, size_t length) {
  Serial.print("Connected to socket.io server:");
  Serial.println(payload);
  
  JsonObject& root = jsonBuffer.createObject();
  root["type"] = "machine";
  root["id"] = machineID;
  root["ver"]= FWVer;
  
  String JSONMsg;
  root.printTo(JSONMsg);

  webSocket.emit("checkingIn", (char*) JSONMsg.c_str());
}

void setup(){
  Serial.begin(115200);
  Serial.println();

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi.");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("ok.");
  webSocket.on("connect", socketIOConnect);
  webSocket.begin(websocketServerURL,80);
  setupOTA();
  
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    if (client.connect("HVAC_ESP8266", mqttUser, mqttPassword )) {
      Serial.println("connected");  
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
  //client.publish("houseData/HVACData", "HVAC ESP8266 online");
  client.subscribe("houseData/SetOilVol");

  EEPROM.begin(512);
  EEPROM.get(volumeEEAddr,volumeInmL);
  EEPROM.commit();
  EEPROM.end();

  String payload = "Volume saved:";
  payload += volumeInmL;
  Serial.println(payload);
  timeClient.begin();
  timeClient.setTimeOffset(0);
  Serial.print("Getting time...");
  while (!timeClient.update()){}
  Serial.println("ok");
  for (int i = 0; i < 3; ++i) {
    digitalWrite(BUILTIN_LED, LOW);
    delay(500);
    digitalWrite(BUILTIN_LED, HIGH);
  }
  lastMillis = millis();
  lastOilFlowMeasTime=millis();
  setupInterrupts();
}

void flowSensorInterrupt(){
}

void checkMQTTConnection(){
  if (!client.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (client.connect("HVAC_ESP8266", mqttUser, mqttPassword )) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected

    client.loop();
  }  
}

void loop() {
  webSocket.loop();
  checkMQTTConnection();
  timeClient.update();
  ArduinoOTA.handle();
}


