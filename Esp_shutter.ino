//Template of firmware for ESP 8266 to control an automated system with MQTT protocol

//Global includes
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Define constants: pin numbers and delays. Warning: do not use PIN GPIO 16 (D0) since it is LED_BUILTIN
#define NO_WIFI   200
#define NO_MQTT   1000
#define WORKING   0
#define PUB_DELAY 30000
#define SSR_DELAY 50
#define ERR       4
#define UPSWITCH  12
#define DOWNSWITCH  14
#define UP        4
#define DOWN      5
#define OPEN_CMD  0
#define CLOSE_CMD 1
#define STOP_CMD  2
#define MAX_TILT  180

//Global constants
// Wifi
const PROGMEM char* WL_SSID = "your_ssid";
const PROGMEM char* WL_PWD = "your_password";

// MQTT
const PROGMEM char* SERVER = "192.168.1.4";
const PROGMEM uint16_t PORT = 1883;
const PROGMEM char* CLIENT_ID = "your_client_id";
const PROGMEM char* USERNAME = "user";
const PROGMEM char* PASSWORD = "password";
const PROGMEM char* STATE_TOPIC = "shutter/status";
const PROGMEM char* COMMAND_TOPIC = "shutter/commands";
const PROGMEM char* TILT_STATE_TOPIC = "shutter/tilt/status";
const PROGMEM char* TILT_COMMAND_TOPIC = "shutter/tilt/commands";

// payload
const char* PAYLOAD_OPEN = "OPEN";
const char* PAYLOAD_CLOSE = "CLOSE";
const char* PAYLOAD_STOP = "STOP";

//Global variables
bool ledStatus = false;
bool wifiRequested = false;
unsigned long nextBlinkMillis = 0;
unsigned long nextStatusMillis = 0;
bool publishStatus = false;
unsigned long blinkInterval = NO_WIFI;
volatile uint8_t command = ERR;
unsigned long prevCmdMillis = 0;
unsigned long stopCmdMillis = 0;
unsigned long tilt = 0;
WiFiClient wfClient;
PubSubClient mqttClient;



//Set up the sketch components
void setup() {
  
  //Serial setup. Serial can be left in the final sketch since the ESP has lots of memory
  Serial.begin(115200);
  
  //Pins initialization
  pinMode(UPSWITCH, INPUT);
  pinMode(DOWNSWITCH, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(UP, OUTPUT);
  pinMode(DOWN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(UP, LOW);
  digitalWrite(DOWN, LOW);

  // Init the mqtt client parameters
  mqttClient.setClient(wfClient);
  mqttClient.setServer(SERVER, PORT);
  mqttClient.setCallback(callback);

  //Led initialization and pattern
  delay(1000);
  digitalWrite(LED_BUILTIN, HIGH);

  //Init timer global variables
  nextBlinkMillis = millis() + blinkInterval;
  nextStatusMillis = millis() + PUB_DELAY;
  prevCmdMillis = 0;
  
}



//Code to run repeatedly
void loop() {

  //Execute the command if registered. Save the command variable to avoid incoherent results caused by its modification
  uint8_t cmd = command;
  command = ERR;
  executeCommand(cmd);
  
  //Check the timers
  checkTimers();
  
  //Check the connectivity
  checkConnectivity();
  
}



//Execute a command
void executeCommand(uint8_t cmd) {
  
  if (cmd == OPEN_CMD) {
    //First stop the shutter
    executeCmd(STOP_CMD);
    unsigned long currMillis = millis();
    unsigned long tiltDelta = MAX_TILT - tilt;
    stopCmdMillis = currMillis + tiltDelta;
    
  } else if (cmd == CLOSE_CMD) {
    
  } else if (cmd == STOP_CMD) {
    //Stop the shutter and insert a safety delay to avoid breaking of the zero-crossing SSRs.
    digitalWrite(UP, LOW);
    digitalWrite(DOWN, LOW);
    delay(50);
    
  }
  
}



//This method checks if the Millis timers have expired. The timers work by saving the Millis value of the event. This way the Millis overflow problem is solved
void checkTimers() {
  
  //Sample the current time
  unsigned long currentMillis = millis();

  //Blink timer handling
  if (blinkInterval == WORKING) { //If the system is working leave the led on
    digitalWrite(LED_BUILTIN, LOW);
    ledStatus = true;
  } else if(currentMillis >= nextBlinkMillis) {
    triggerLed();
    nextBlinkMillis = currentMillis + blinkInterval;
  }

  //Publish status timer handling
  if(currentMillis >= nextStatusMillis) {
    publishStatus = true;
    nextStatusMillis = currentMillis + PUB_DELAY;
  }
  
}



//This method checks that the wifi is connected and sets the led blink time if there is anything wrong
void checkConnectivity() {

  //Check if the wifi is not connected
  if(WiFi.status() != WL_CONNECTED) {
    if(!wifiRequested) {
      WiFi.begin(WL_SSID, WL_PWD);
      blinkInterval = NO_WIFI;
      wifiRequested = true;
    }
  } else {
    wifiRequested = false;
    //Check if MQTT is not connected
    if (!mqttClient.connected()) {
      if(mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD)) {
        mqttClient.subscribe(COMMAND_TOPIC);
        /* Please add topics */
      }
      blinkInterval = NO_MQTT;
    } else {
      blinkInterval = WORKING;
      //Publish the status
      if(publishStatus) {
        publishStatus = false;
        mqttClient.publish(STATE_TOPIC, PAYLOAD_ON);
        /* Please add publish statements */
      }
      //Loop the client
      mqttClient.loop();
    }
  }
  
}



//Callback to receive MQTT commands. This only sets the command voltaile variable.
void callback(char* topic, byte* payload, unsigned int len) {
  
  //Parse the message
  char message_buff[len+1];
  int i = 0;
  for(i=0; i<len; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  //Check the topic and the command
  if(strcmp(topic, COMMAND_TOPIC) == 0) {
    if(strcmp(message_buff, PAYLOAD_OPEN) == 0) {
      command = OPEN_CMD;
    } else if(strcmp(message_buff, PAYLOAD_CLOSE) == 0) {
      command = CLOSE_CMD;
    } else if(strcmp(message_buff, PAYLOAD_STOP) == 0) {
      command = STOP_CMD;
    }
  }
  
}



//This method triggers the LED_BUILTIN. Note that the LED is on when LOW is set
void triggerLed() {
  if(ledStatus) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledStatus = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    ledStatus = true;
  }
  
}

