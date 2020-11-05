#include <FS.h>                   //this needs to be first, or it all crashes and burns...

#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space

#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#include <BlynkSimpleEsp8266.h>
//needed for library
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <SimpleTimer.h>
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson
#include <ArduinoOTA.h>       // for local OTA updates
//start DRD
#include <DoubleResetDetector.h>
// Number of seconds after reset during which a
// subseqent reset will be considered a double reset.
#define DRD_TIMEOUT 10
// RTC Memory Address for the DoubleResetDetector to use
#define DRD_ADDRESS 0
DoubleResetDetector drd(DRD_TIMEOUT, DRD_ADDRESS);
//end
SimpleTimer timer;
WidgetLED wledSTAT(V5);

//define your default values here, if there are different values in config.json, they are overwritten.
char mqtt_server[40] = "blynk-cloud.com";
char mqtt_port[6] = "80";
char blynk_token[34] = "7WZ8hKTOM_abcU6hMoUd1NrfnjIZ3CC2";

//flag for saving data
bool shouldSaveConfig = false;
// KENA TUKAR IKUT KERETA SEBELUM UPLOAD
char OTAhost[] = "LFR";              // Optional.

bool isFirstConnect = true;

//for LED status
#include <Ticker.h>
Ticker ticker;

#ifndef LED_BUILTIN
#define LED_BUILTIN 16 // ESP32 DOES NOT DEFINE LED_BUILTIN
#endif
int LED = LED_BUILTIN;

//---------LFR----
#define MAKERLINE_AN  A0

#define MAX_SPEED 550

int PWMA = 5; //motor kiri
int PWMB = 4; //motor kanan
int DA = 0; //motor kiri
int DB = 2; //motor kanan

int motorB ; // motor kanan
int motorA ; // motor kiri

float Kp = 1.5; //ubah
float Kd = 5; //ubah

int adcMakerLine = 0;
int adcSetPoint = 0;
int proportional = 0;
int lastProportional = 0;
int derivative = 0;
int powerDifference = 0;
int motorLeft = 0;
int motorRight = 0;
unsigned long currentMillis = 0;
unsigned long previousMillis = 0;
const int interval = 10;
//----------------------------------

void tick()
{
  //toggle state
  digitalWrite(LED, !digitalRead(LED));     // set pin to the opposite state
}

void reconnectBlynk() {                         // reconnect to server if disconnected
  if (!Blynk.connected()) {
    ticker.attach(0.2, tick);
    if (Blynk.connect()) {
      ticker.detach();
      BLYNK_LOG("Reconnected");
    } else {
      BLYNK_LOG("Not reconnected");
    }
  }
}


BLYNK_CONNECTED() {
  if (isFirstConnect) {
    Blynk.syncAll();
    // Blynk.notify("LET'S GO MATE!!!!");
    isFirstConnect = false;
  }
}

void blinkSTATLedWidget()
{
  if (wledSTAT.getValue()) {
    //send off to led controller
    wledSTAT.off();
  } else {
    //send on to led controller
    wledSTAT.on();
  }
}



//callback notifying us of the need to save config
void saveConfigCallback () {
  Serial.println(F("Should save config"));
  shouldSaveConfig = true;
}

void configModeCallback (WiFiManager *myWiFiManager)
{
  Serial.println(F("Entered config mode"));
  Serial.println(WiFi.softAPIP());
  //entered config mode, make led toggle faster
  ticker.attach(0.2, tick);
  drd.stop();
}


void setup() {
  pinMode(MAKERLINE_AN, INPUT);
  // analogReadResolution(10);

  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DA, OUTPUT);
  pinMode(DB, OUTPUT);

  analogWrite(PWMA, 0);
  digitalWrite(DA, 0);
  analogWrite(PWMB, 0);
  digitalWrite(DB, 0);

  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println(F("\n Starting"));

  //set led pin as output
  pinMode(LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(0.6, tick);
  //clean FS, for testing
  //SPIFFS.format();

  //read configuration from FS json
  Serial.println(F("mounting FS..."));

  if (SPIFFS.begin()) {
    Serial.println(F("mounted file system"));
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println(F("reading config file"));
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println(F("opened config file"));
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println(F("\nparsed json"));

          strcpy(mqtt_server, json["mqtt_server"]);
          strcpy(mqtt_port, json["mqtt_port"]);
          strcpy(blynk_token, json["blynk_token"]); // remote_token

        } else {
          Serial.println(F("failed to load json config"));
        }
        configFile.close();
      }
    }
  } else {
    Serial.println(F("failed to mount FS"));
  }
  //end read



  // The extra parameters to be configured (can be either global or just in the setup)
  // After connecting, parameter.getValue() will get you the configured value
  // id/name placeholder/prompt default length
  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", mqtt_server, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", mqtt_port, 6);
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 32);

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  wifiManager.setAPCallback(configModeCallback);
  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //set static ip
  //  wifiManager.setSTAStaticIPConfig(IPAddress(10, 0, 1, 99), IPAddress(10, 0, 1, 1), IPAddress(255, 255, 255, 0));

  //add all your parameters here
  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  wifiManager.addParameter(&custom_blynk_token);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //set minimu quality of signal so it ignores AP's under that quality
  //defaults to 8%
  //wifiManager.setMinimumSignalQuality();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(180);

  //fetches ssid and pass and tries to connect
  //if it does not connect it starts an access point with the specified name
  //here  "AutoConnectAP"
  //and goes into a blocking loop awaiting configuration
  if (drd.detectDoubleReset()) {
    Serial.println(F("Double Reset Detected"));
    // wifiManager.startConfigPortal(OTAhost);
    wifiManager.resetSettings();
    ESP.reset();
    /*  wifiManager.setConfigPortalTimeout(180);
      if (!wifiManager.startConfigPortal(OTAhost)) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        //reset and try again, or maybe put it to deep sleep
        //ESP.restart();
        ESP.reset();
        delay(5000);
      }*/
  } else

    if (!wifiManager.autoConnect(OTAhost)) {
      Serial.println(F("failed to connect and hit timeout"));
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.reset();
      // ESP.restart();
      delay(5000);
    }

  //if you get here you have connected to the WiFi
  Serial.println(F("connected...yeey :)"));
  ticker.detach();
  //keep LED on
  digitalWrite(LED, LOW);
  drd.stop();
  //read updated parameters
  strcpy(mqtt_server, custom_mqtt_server.getValue());
  strcpy(mqtt_port, custom_mqtt_port.getValue());
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println(F("saving config"));
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["mqtt_server"] = mqtt_server;
    json["mqtt_port"] = mqtt_port;
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println(F("failed to open config file for writing"));
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }

  //Serial.println("local ip");
  //Serial.println(WiFi.localIP());
  Blynk.config(blynk_token, mqtt_server, 80);
  bool result = Blynk.connect();
  int mytimeout = millis() / 1000;
  while (Blynk.connect(1000) == false) {        // wait here until connected to the server
    if ((millis() / 1000) > mytimeout + 8) {    // try to connect to the server for less than 9 seconds
      Serial.println(F("BLYNK Connection Fail"));
      Serial.println(blynk_token);
      wifiManager.resetSettings();
      ESP.reset();
      //ESP.restart();
      //  delay (5000);
      /*   wifiManager.setConfigPortalTimeout(180);
        if (!wifiManager.startConfigPortal(OTAhost)) {
         Serial.println("failed to connect and hit timeout");
         delay(3000);
         //reset and try again, or maybe put it to deep sleep
         //ESP.restart();
         ESP.reset();
         delay(5000);
        }*/
    }
    else
    {
      Serial.println(F("BLYNK Connected"));
    }
  }

  Serial.println(blynk_token);
  ArduinoOTA.setHostname(OTAhost);              // for local OTA updates
  ArduinoOTA.begin();

  timer.setInterval(15000, reconnectBlynk); // check every 15 seconds if we are connected to the server
  timer.setInterval(2000, blinkSTATLedWidget);

  // Place robot at the center of line
  adcSetPoint = analogRead(MAKERLINE_AN);
  delay(2000);
}



BLYNK_WRITE(V4)//      slider  from 600 to 1023!!!!
{
  float vel = param.asInt();
  Kp = vel;
}
/*
  BLYNK_WRITE(V5)//      slider  from 600 to 1023!!!!
  {
  int speedz = param.asInt();
  MAX_SPEED = speedz;
  }
*/

BLYNK_WRITE(V6)//      slider  from 600 to 1023!!!!
{
  float fac = param.asInt();
  Kd = fac;
}

BLYNK_READ(V10)//      slider  from 600 to 1023!!!!
{
  Blynk.virtualWrite(V10, Kp);
}

BLYNK_READ(V11)//      slider  from 600 to 1023!!!!
{
  Blynk.virtualWrite(V11, Kd);
}



void loop() {

  if (Blynk.connected()) {   // to ensure that Blynk.run() function is only called if we are still connected to the server
    Blynk.run();
  }
  timer.run();
  ArduinoOTA.handle();       // for local OTA updates

  //--------LFR--------------

  adcMakerLine = analogRead(MAKERLINE_AN);


  proportional = adcMakerLine - adcSetPoint;

  derivative = proportional - lastProportional;

  lastProportional = proportional;

  powerDifference = (proportional * Kp) + (derivative * Kd);


  motorLeft = MAX_SPEED + powerDifference;

  motorRight = MAX_SPEED - powerDifference;

  motorLeft = constrain(motorLeft, 0, MAX_SPEED);
  motorRight = constrain(motorRight, 0, MAX_SPEED);

  digitalWrite(DA, LOW); //FORWARD
  analogWrite(PWMA, motorLeft);
  digitalWrite(DB, LOW); //FORWARD
  analogWrite(PWMB, motorRight);

}
// }

//----------------------------
