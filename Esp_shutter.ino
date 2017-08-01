//Firmware that controls an automated shutter. Since the up and down lines should not be activated simultaneosly, the controller forces a short delay to ensure tht the SSR switches at the right point.

//Global includes
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

//Define constants
#define SWITCHUP  12
#define SWITCHDOWN   14
#define LIGHT     4
#define NO_WIFI   5000
#define NO_MQTT   500
#define WORKING   10000
#define DELAY     100

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
const char* LIGHT_ON = "ON";
const char* LIGHT_OFF = "OFF";

//Global variables
bool lightStatus = false;
bool ledStatus = false;
bool wifiRequested = false;
bool publishStatus = true;
unsigned long prevBlinkMillis = 0;
unsigned long blinkInterval = 0;
unsigned long prevDebounceMillis = 0;
volatile uint8_t command = 0;

WiFiClient wfClient;
PubSubClient mqttClient;

void setup() {
  Serial.begin(115200);
  // Init the pins
  pinMode(SWITCHUP, INPUT);
  pinMode(SWITCHDOWN, INPUT);
  pinMode(LIGHT, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(LIGHT, HIGH);
  delay(500);
  digitalWrite(LED_BUILTIN, HIGH);
  
  //Init the global variables
  lightStatus = false;
  ledStatus = true;
  readSwitches();
  blinkInterval = NO_WIFI;

  // Init the mqtt client parameters
  mqttClient.setClient(wfClient);
  mqttClient.setServer(SERVER, PORT);
  mqttClient.setCallback(callback);

  //Safety initial delay
  delay(500);
  
  //Set the starting point for flashing
  prevBlinkMillis = millis();
  prevDebounceMillis = millis();
}

void loop() {
  //Check if there is something to do
  checkEverything();
  //A flashing speed has been set. Follow it.
  unsigned long tempMillis = millis();
  //Millis overflow check
  if(tempMillis < prevBlinkMillis) {
    prevBlinkMillis = 0;
  }

  //Blink timer handling
  if((tempMillis-prevBlinkMillis) >= blinkInterval) {
    triggerLed();
    prevBlinkMillis = millis();
    if(blinkInterval == WORKING) {
      publishStatus = true;
    }
  }

  //Check if the switches status has changed
  bool manualChange = readSwitches(); //This sets the status of the switches to the new one
  if(manualChange) {
    prevDebounceMillis = tempMillis;
    debounceInterval = DEBOUNCE_INT;
  }

  if(debounceInterval != 0 && (tempMillis-prevDebounceMillis) > debounceInterval) { //Equal removed because if the status changes, the timer is set to tempMillis
    triggerLight(!lightStatus);
    publishStatus = true;
    debounceInterval = 0;
    Serial.println("SWITCH TRIGGER");
  }
  
}

//This method checks that everything is in order
void checkEverything() {
  //First check if any MQTT command was received
  if (command == 1) {
    triggerLight(true);
    command = 0;
    publishStatus = true;
  } else if (command == 2) {
    triggerLight(false);
    command = 0;
    publishStatus = true;
  }

  //Check if the wifi is not connected
  if(WiFi.status() != WL_CONNECTED) {
    if(!wifiRequested) {
      WiFi.begin(WL_SSID, WL_PWD);
      blinkInterval = NO_WIFI;
      wifiRequested = true;
    }
  } else {
    wifiRequested = false;
    //Check if MQTT is connected
    if (!mqttClient.connected()) {
      if(mqttClient.connect(CLIENT_ID, USERNAME, PASSWORD)) {
        mqttClient.subscribe(COMMAND_TOPIC);
      }
      blinkInterval = NO_MQTT;
    } else {
      blinkInterval = WORKING;
      //Publish the status
      if(publishStatus) {
        publishStatus = false;
        if(lightStatus) {
          mqttClient.publish(STATE_TOPIC, LIGHT_ON);
        } else {
          mqttClient.publish(STATE_TOPIC, LIGHT_OFF);
        }
      }
      //Loop the client
      mqttClient.loop();
    }
  }
}

//Callback to receive MQTT commands
//Note that no command is executed here, it is only set
void callback(char* topic, byte* payload, unsigned int len) {
  
  //Parse the message
  char message_buff[len+1];
  int i = 0;
  for(i=0; i<len; i++) {
    message_buff[i] = payload[i];
  }
  message_buff[i] = '\0';
  
  String msgString = String(message_buff);

  
  //Ignore the topic since it is always the same
  if (msgString.equals("ON")) {
    command = 1;
  } else if (msgString.equals("OFF")) {
    command = 2;
  }
}


//Read the switches and set the status
bool readSwitches() {
  bool ret = false;
  int temp1 = digitalRead(SWITCH1);
  int temp2 = digitalRead(SWITCH2);
  Serial.print(temp1);
  Serial.print("     ");
  Serial.println(temp2);
  if(temp1 != switch1 || temp2 != switch2) {
    ret = true;
  }
  switch1 = temp1;
  switch2 = temp2;
  return ret;
}

void triggerLed() {
  if(ledStatus) {
    digitalWrite(LED_BUILTIN, HIGH);
    ledStatus = false;
  } else {
    digitalWrite(LED_BUILTIN, LOW);
    ledStatus = true;
  }
}

void triggerLight(bool light_on) {
  if(light_on) {
    digitalWrite(LIGHT, LOW);
    lightStatus = true;
  } else {
    digitalWrite(LIGHT, HIGH);
    lightStatus = false;
  }
}

