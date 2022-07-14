#include <Arduino.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <FlashStorage.h>
#include <RunningMedian.h>
#include "WifiSecurity.h"

/******************************************************************************
 * Begin Wifi Setup
 *****************************************************************************/

int status = WL_IDLE_STATUS;     // the Wifi radio's status
WiFiClient netclient;
/******************************************************************************
 * End Wifi Setup
 *****************************************************************************/

/******************************************************************************
 * Begin MQTT Setup
 *****************************************************************************/
PubSubClient mqttclient(netclient);
IPAddress server(192, 168, 68, 111);
long lastReconnectAttempt = 0;
unsigned long keepalivetime = 0;
int deltaKeepAliveAcks = 0;
bool needAck = true;

char mqttdev[] = "HauptWasserZaehler";
char mqtt_devmodechan[] = "leo26/wasserzaehler/hauptzaehler/setup/devmode";
char mqtt_triggermodechan[] = "leo26/wasserzaehler/hauptzaehler/setup/triggermode";
char mqtt_uprangechan[] = "leo26/wasserzaehler/hauptzaehler/setup/uppertrigger";
char mqtt_lowrangechan[] = "leo26/wasserzaehler/hauptzaehler/setup/lowertrigger";
const char mqtt_setup_needAck[] = "leo26/wasserzaehler/hauptzaehler/setup/NeedAck";

char mqtt_triggerchan[] = "leo26/wasserzaehler/hauptzaehler/trigger";
char mqtt_rawchan[] = "leo26/wasserzaehler/hauptzaehler/raw";

char mqtt_devmodechanack[] = "leo26/wasserzaehler/hauptzaehler/devmode";
char mqtt_triggermodechanack[] = "leo26/wasserzaehler/hauptzaehler/triggermode";
char mqtt_uprangechanack[] = "leo26/wasserzaehler/hauptzaehler/uppertrigger";
char mqtt_lowrangechanack[] = "leo26/wasserzaehler/hauptzaehler/lowertrigger";
char mqtt_reconnectcount[] = "leo26/wasserzaehler/hauptzaehler/reconnectcount";
const char mqtt_keepaliveack[] = "leo26/wasserzaehler/hauptzaehler/ack/keepalive";

char mqtt_keepalive[] = "leo26/wasserzaehler/hauptzaehler/keepalive";

/******************************************************************************
 * End MQTT Setup
 *****************************************************************************/

const int analogInPin = A7;  // Analog input pin that the photo transistor is attached to
const int irOutPin = 2; // Digital output pin that the IR-LED is attached to
const int ledOutPin = 13; // Signal LED output pin

int sensorValueOff = 0;  // value read from the photo transistor when ir LED is off
int sensorValueOn = 0;  // value read from the photo transistor when ir LED is on
int sensorValue = 0; // difference sensorValueOn - sensorValueOff
float filteredValue; // filtered sensor value

// definitions for low pass filter
float filterAlpha = 0.1f;
float filterOut = 0;
boolean filterLoad = true;

int RawOperationMode = 0;
int TriggerOperationMode = 1;

// trigger state and level
float triggerLevelLow;
float triggerLevelHigh;
boolean triggerState = false;

// Address of trigger levels in EEPROM
FlashStorage(triggerLevelLowAddr, int);
FlashStorage(triggerLevelHighAddr, int);
FlashStorage(RawModeAddr, int);
FlashStorage(TriggerModeAddr, int);

RunningMedian lowestValuesToTrigger = RunningMedian(50);
RunningMedian highestValuesToTrigger = RunningMedian(50);
float lastValLow = 9999;
float lastValHigh = 0;

/**
 * Read trigger levels from EEPROM
 */
void readTriggerLevels() {
  triggerLevelLow = triggerLevelLowAddr.read();
  triggerLevelHigh = triggerLevelHighAddr.read();
  Serial.print("Trigger levels: ");
  Serial.print(triggerLevelLow);
  Serial.print(" ");
  Serial.println(triggerLevelHigh);
}

void readOperationMode() {
  RawOperationMode = RawModeAddr.read();
  TriggerOperationMode = TriggerModeAddr.read();
  Serial.print("Operation modes:" );
  Serial.print("Raw: " + RawOperationMode );
  Serial.println("Trigger: " + TriggerOperationMode);
}

/**
 * Detect and print a trigger event
 */
void detectTrigger(float val) {
  boolean nextState = triggerState;
  if (val > triggerLevelHigh) {
    nextState = true;
  } else if (val < triggerLevelLow) {
    nextState = false;
  }
  if (nextState != triggerState) {
    triggerState = nextState;
    if (triggerState) {
      mqttclient.publish(mqtt_triggerchan, "1");
    } else {
      mqttclient.publish(mqtt_triggerchan, "0");
    }
    // control internal LED
    digitalWrite(ledOutPin, triggerState);
  }
}

void automodifyTriggers(float val) {
  if (val < lastValLow ) {
    lastValLow = val;
  }
  if (val > (lastValHigh - lastValLow)/2 && lastValHigh != 0 && lastValLow != 9999) {
    lowestValuesToTrigger.add(lastValLow);
    lastValLow = 9999;
  }
  if (val > lastValHigh) {
    lastValHigh = val;
  }
  if (val < (lastValHigh - lastValLow)/2 && lastValHigh != 0 && lastValLow != 9999) {
    highestValuesToTrigger.add(lastValHigh);
    lastValHigh = 0;
  }
  if (lowestValuesToTrigger.getCount() == 50 && highestValuesToTrigger.getCount() == 50) {
    float lowerAverage = lowestValuesToTrigger.getAverage();
    float higherAverage = highestValuesToTrigger.getAverage();

    float divlowHigh = higherAverage - lowerAverage;
    float triggerLevelHighNew = higherAverage - divlowHigh * 0.3;
    float triggerLevelLowNew = lowerAverage + divlowHigh * 0.3; 

    char str[100];
    if (abs(triggerLevelLowNew - triggerLevelLow)/triggerLevelLowNew > 0.1) {
      triggerLevelLow = triggerLevelLowNew;
      triggerLevelLowAddr.write(triggerLevelLow);
      itoa((int)triggerLevelLow, str, 10);
      mqttclient.publish(mqtt_lowrangechanack, str);
    }

    if (abs(triggerLevelHighNew - triggerLevelHigh)/triggerLevelHighNew > 0.1) {
      triggerLevelHigh = triggerLevelHighNew;
      triggerLevelHighAddr.write(triggerLevelHigh);
      itoa((int)triggerLevelHigh, str, 10);
      mqttclient.publish(mqtt_uprangechanack,str);
    }
  }
}

/* 
 * Low pass filter to eleminate spikes
 */
float lowpass(int value) {
  if (filterLoad) {
    filterOut = value;
    filterLoad = false;
  }
  filterOut = filterAlpha * value + (1.f - filterAlpha) * filterOut;
  return filterOut;
}

/******************************************************************************
 * Begin Wifi Functions
 *****************************************************************************/
void printMacAddress(byte mac[]) {
  for (int i = 5; i >= 0; i--) {
    if (mac[i] < 16) {
      Serial.print("0");
    }
    Serial.print(mac[i], HEX);
    if (i > 0) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void printWifiData() {
  // print your board's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
  printMacAddress(mac);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  printMacAddress(bssid);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}
/******************************************************************************
 * End Wifi Functions
 *****************************************************************************/

/******************************************************************************
 * Begin MQTT Functions
 *****************************************************************************/
void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  String msgvalue;
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i=0;i<length;i++) {
    msgvalue.concat((char)payload[i]);
  }
  Serial.println(msgvalue);

  if (strcmp(topic,mqtt_lowrangechan) == 0) {
      triggerLevelLowAddr.write(msgvalue.toInt());
      triggerLevelLow = msgvalue.toInt();
      Serial.print("Set Lower Trigger to ");
      Serial.println(msgvalue.toInt());
      mqttclient.publish(mqtt_lowrangechanack,msgvalue.c_str());
  } else if (strcmp(topic, mqtt_uprangechan) == 0) {
      triggerLevelHighAddr.write(msgvalue.toInt());
      triggerLevelHigh =msgvalue.toInt();
      Serial.print("Set Upper Trigger to ");
      Serial.println(msgvalue.toInt());
      mqttclient.publish(mqtt_uprangechanack,msgvalue.c_str());
  } else if (strcmp(topic, mqtt_triggermodechan) == 0) {
      TriggerModeAddr.write(msgvalue.toInt());
      TriggerOperationMode = msgvalue.toInt();
      Serial.print("Set Triggermode to ");
      Serial.println(msgvalue.toInt());
      mqttclient.publish(mqtt_triggermodechanack,msgvalue.c_str());
  } else if (strcmp(topic, mqtt_devmodechan) == 0) {
      RawModeAddr.write(msgvalue.toInt());
      RawOperationMode = msgvalue.toInt();
      Serial.print("Set Devmode to ");
      Serial.println(msgvalue.toInt());
      mqttclient.publish(mqtt_devmodechanack,msgvalue.c_str());
  }
  else if (strcmp(topic, mqtt_keepaliveack) == 0)
  {
    //Got an ack - reset counter
    deltaKeepAliveAcks = 0;
  }
  else if (strcmp(topic, mqtt_setup_needAck) == 0)
  {
    if (msgvalue.toInt() == 0)
    {
      needAck = false;
    }
    else
    {
      needAck = true;
    }
  }
   else {
      Serial.println("Unknown topic! No Action taken!");
  }
}

boolean mqttreconnect() {
  digitalWrite(ledOutPin, HIGH); //We are not connected.
  status=WiFi.status();
  if(status==WL_CONNECTION_LOST || status==WL_DISCONNECTED || status==WL_SCAN_COMPLETED ||status==WL_CONNECT_FAILED) {
    WiFi.end();
    WiFi.disconnect();
  }
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }
  if (mqttclient.connect(mqttdev)) {
    // Once connected, publish an announcement...
    //client.publish("outTopic","hello world");
    // ... and resubscribe
    mqttclient.subscribe(mqtt_devmodechan);
    mqttclient.subscribe(mqtt_triggermodechan);
    mqttclient.subscribe(mqtt_uprangechan);
    mqttclient.subscribe(mqtt_lowrangechan);
    mqttclient.subscribe(mqtt_keepaliveack);
    digitalWrite(ledOutPin, LOW); //We are connected.
    mqttclient.publish(mqtt_reconnectcount,"1");
  }
  return mqttclient.connected();
}

/******************************************************************************
 * End MQTT Functions
 *****************************************************************************/


/**
 * Setup.
 */
void setup() {
  
  Serial.begin(9600);
  // while (!Serial) {
  //   ; // wait for serial port to connect. Needed for native USB port only
  // }

  mqttclient.setServer(server, 1883);
  mqttclient.setCallback(mqtt_callback);
  
  /****************************************************************************
   * Wifi Setup
   ***************************************************************************/
  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // attempt to connect to Wifi network:
  while (status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(10000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  delay(1500);
  lastReconnectAttempt = 0;

  // initialize the digital pins as an output.
  pinMode(irOutPin, OUTPUT);
  pinMode(ledOutPin, OUTPUT);
  digitalWrite(ledOutPin, LOW); //We are connected.

  // read config from EEPROM
  readTriggerLevels();

  //TriggerModeAddr.write(0);
  //RawModeAddr.write(1);
  // read mode from EEPROM
  readOperationMode();

  mqttclient.publish(mqtt_lowrangechanack,String(triggerLevelLow).c_str());
  mqttclient.publish(mqtt_uprangechanack,String(triggerLevelHigh).c_str());
  mqttclient.publish(mqtt_triggermodechanack,String(TriggerOperationMode).c_str());
  mqttclient.publish(mqtt_devmodechanack,String(RawOperationMode).c_str());
}

void loop() {
  if (keepalivetime < millis()) {
    deltaKeepAliveAcks += 1;
    mqttclient.publish(mqtt_keepalive,"1");
    keepalivetime = millis() + 60000;
  }
  if (deltaKeepAliveAcks >= 5 && needAck)
    {
      //5 Keepalives without Answer from Node-Red
      NVIC_SystemReset();
    }
  // put your main code here, to run repeatedly:
  if (!mqttclient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      // Attempt to reconnect
      if (mqttreconnect()) {
        lastReconnectAttempt = 0;
      }
    }
  } else {
    // Client connected
    // perform measurement
    // turn IR LED off
    digitalWrite(irOutPin, LOW);
    // wait 10 milliseconds
    delay(10);
    // read the analog in value:
    sensorValueOff = analogRead(analogInPin);           
    // turn IR LED on
    digitalWrite(irOutPin, HIGH);
    delay(10);
    // read the analog in value:
    sensorValueOn = analogRead(analogInPin);
    sensorValue = sensorValueOn - sensorValueOff;
    filteredValue = lowpass(sensorValue);

    if (RawOperationMode == 1) {
      char buffer[64];
      int ret = snprintf(buffer, sizeof buffer, "%f", filteredValue);
      if (ret >= 0  && ret < sizeof(buffer)) {
        //Only publish if float fits into buffer
        mqttclient.publish(mqtt_rawchan,buffer);
      }
      // Serial.println(filteredValue);
    }
    automodifyTriggers(filteredValue);

    if (TriggerOperationMode == 1) {
      detectTrigger(filteredValue);
    }
    delay(10);  
    mqttclient.loop();
  }
}