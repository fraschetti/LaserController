#include "WiFiEsp.h"
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <TimerThree.h>
#include <arduino-timer.h>
#include "SerialStringReader.h"
#include "secrets.h"

//TODO reinit Wifi completely ever N hours?
//TODO add failed publish counter
//TODO add relay (with decent debounce) for the chiller
//TODO add variable fan control
//TODO publish messages on sleep(s) and wake
//TODO stagger wake up to limit the in-rush current?
//TODO If a job starts and we're asleep, wake up

/*
  Origial protoype from:
  https://forum.arduino.cc/t/troubles-connecting-esp8266-01-with-arduino-mega-2560/646624/10

  Minimal wiring is:

  ESP TX -----To Mega RX (Serial 1)
  ESP CH_PD - To 3.3v power supply's +
  ESP RST --- Can leave open (although I tie it to the 3.3v power supply's + with a 10kOhm resistor)
  ESP VCC --- To 3.3v power supply's +

  ESP GND --- Connect it to both the Mega's GND, and also to the 3.3v power supply's GND
  GPIO2 ----- Can leave open
  GPIO0 ----- Can leave open (although I tie it to the 3.3v power supply's + with a 10kOhm resistor)
  ESP RX -----To Mega TX (Serial 1)

  I connect the RST and GPIO0 pins to the 3.3v power supply's + with the 10kOhm resistors, so those inputs aren't floating. Since one is the Reset, and the other is the Program pin.

  For initial testing, I see you are using the Arduino Mega's 3.3v pin as the power supply. This really isn't a great idea, as the ESP8266 chip can pull a bit of power when transmitting. So while it may be ok during your learning phase of just trying to the two talking to each other, you really need a separate 3.3v power supply for the ESP chip.
*/

// ********************************************************* DEFINITIONS *********************************************************
volatile bool DEBUG = false;

volatile int wifiStatus = WL_IDLE_STATUS;
volatile bool wifiInitInProgress = false;
volatile bool mqttInitInProgress = false;

const char* TRUE_STR = "true";
const char* FALSE_STR = "false";
const char* SLEEP_MODE_AWAKE = "awake";
const char* SLEEP_MODE_LIGHT = "light";
const char* SLEEP_MODE_DEEP = "deep";

volatile unsigned long seqNo = 0;

const int INPUT_LASER_KEY = 47;
const int INPUT_STATUS = 45;
const int INPUT_WIND = 43;
const int INPUT_IDLE = 41;
const int INPUT_ERROR = 39;
const int INPUT_FLOW_SENSOR = 37;
const int INPUT_CHILLER_SENSOR = 35;
const int INPUT_DOOR_SENSOR = 33;  //closed == HIGH

const int INPUT_LASER_KEY_IDX = 0;
const int INPUT_STATUS_IDX = 1;
const int INPUT_WIND_IDX = 2;
const int INPUT_IDLE_IDX = 3;
const int INPUT_ERROR_IDX = 4;
const int INPUT_FLOW_SENSOR_IDX = 5;
const int INPUT_CHILLER_SENSOR_IDX = 6;
const int INPUT_DOOR_SENSOR_IDX = 7;

const int INPUT_PIN_COUNT = 8;
const int INPUT_PINS[] = { INPUT_LASER_KEY, INPUT_STATUS, INPUT_WIND, INPUT_IDLE,
                           INPUT_ERROR, INPUT_FLOW_SENSOR, INPUT_CHILLER_SENSOR, INPUT_DOOR_SENSOR };
volatile int INPUT_STATES[] = { LOW, LOW, LOW, LOW, LOW, LOW, LOW, LOW };

const int RELAY_RED_DOT_PRIMARY = 4;
const int RELAY_RED_DOT_SECONDARY = 5;
const int RELAY_LASER_POWER = 6;
const int RELAY_CHILLER = 7;
const int RELAY_AIR_PRIMARY = 8;
const int RELAY_AIR_LOW = 9;
const int RELAY_AIR_HIGH = 10;
const int RELAY_UNUSED_1 = 11;

const int RELAY_RED_DOT_PRIMARY_IDX = 0;
const int RELAY_RED_DOT_SECONDARY_IDX = 1;
const int RELAY_LASER_POWER_IDX = 2;
const int RELAY_CHILLER_IDX = 3;
const int RELAY_AIR_PRIMARY_IDX = 4;
const int RELAY_AIR_LOW_IDX = 5;
const int RELAY_AIR_HIGH_IDX = 6;
const int RELAY_UNUSED_1_IDX = 7;

const int DEFAULT_ON_DEBOUNCE_MS = 500;
const int DEFAULT_OFF_DEBOUNCE_MS = 500;
const int CHILLER_ON_DEBOUNCE_MS = 15 * 1000;
const int CHILLER_OFF_DEBOUNCE_MS = 15 * 1000;
const int LASER_ON_DEBOUNCE_MS = 5 * 1000;
const int LASER_OFF_DEBOUNCE_MS = 250;

const int RELAY_PIN_COUNT = 8;
const int RELAY_PINS[] = { RELAY_RED_DOT_PRIMARY, RELAY_RED_DOT_SECONDARY, RELAY_LASER_POWER,
                           RELAY_CHILLER, RELAY_AIR_PRIMARY, RELAY_AIR_LOW, RELAY_AIR_HIGH, RELAY_UNUSED_1 };
volatile int RELAY_ON_DEBOUNCE[] = { DEFAULT_ON_DEBOUNCE_MS, DEFAULT_ON_DEBOUNCE_MS, LASER_ON_DEBOUNCE_MS,
                                     CHILLER_ON_DEBOUNCE_MS, DEFAULT_ON_DEBOUNCE_MS, DEFAULT_ON_DEBOUNCE_MS,
                                     DEFAULT_ON_DEBOUNCE_MS, DEFAULT_ON_DEBOUNCE_MS };
volatile int RELAY_OFF_DEBOUNCE[] = { DEFAULT_OFF_DEBOUNCE_MS, DEFAULT_OFF_DEBOUNCE_MS, LASER_OFF_DEBOUNCE_MS,
                                      CHILLER_OFF_DEBOUNCE_MS, DEFAULT_OFF_DEBOUNCE_MS, DEFAULT_OFF_DEBOUNCE_MS,
                                      DEFAULT_OFF_DEBOUNCE_MS, DEFAULT_OFF_DEBOUNCE_MS };
volatile int RELAY_STATES[] = { HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH, HIGH };
volatile long RELAY_LAST_SWITCH[] = { -1, -1, -1, -1, -1, -1, -1, -1 };

/* Adapted from the implementation referced below
  Sine fade
  Produces a sinusoidal fade curve
  created 30 Jan 2019
  modified 9 June 2019
  by Tom Igoe and lighting & interactivity 2019 class
*/
volatile int ledsFadeCurrentLevel = 255;
volatile int ledsTargetConstantLevel = 0;
volatile int ledsFadeChange;
volatile int ledDimCyclesRemaining;
volatile byte ledsLevelTable[256];  // pre-calculated PWM levels

const int LED_DRIVER_PIN = 12;

const int LED_MODE_NA = 0;
const int LED_MODE_FULL = 1;
const int LED_MODE_DIM = 2;
const int LED_MODE_OFF = 3;
const int LED_MODE_QUICK_FLASH_THEN_FULL = 4;
const int LED_MODE_SLOW_FLASH_THEN_DIM = 5;
const int LED_MODE_SLOW_FLASH_THEN_OFF = 6;

volatile int CURRENT_LED_MODE = LED_MODE_OFF;
volatile int NEW_LED_MODE = LED_MODE_FULL;

volatile boolean CURRENT_DOOR_CLOSED = true;

auto timer = timer_create_default();  // create a timer with default settings

const int LIGHT_SLEEP_IDLE_TIME = 10 * 60;  //10 minutes
const int DEEP_SLEEP_IDLE_TIME = 20 * 60;   //20 minutes
volatile unsigned long laserIdleStart = -1;
volatile bool inLightSleep = false;
volatile bool inDeepSleep = false;

const int POST_JOB_AIR_DELAY = 5;  //seconds

volatile unsigned long jobStartTime = -1;
volatile unsigned long jobEndTime = -1;

const int JSON_BUFFER_SIZE = 500;
char JSON_BUFFER[JSON_BUFFER_SIZE];

WiFiEspClient espClient;
PubSubClient mqttClient(espClient);

//SerialStringReader
SerialStringReader serialCommunicator;
Stream& SerialInput = Serial;
Stream& SerialOutput = Serial;
//SerialStringReader

void setup() {
  Serial.begin(115200);   // initialize serial 0 for debugging
  Serial1.begin(115200);  // initialize serial 1 for ESP module

  Serial.println("--- Setup Started ---");

  Serial.println("Initializing replay pin states");
  //Set default pin state leveraging the internal resistors
  for (int i = 0; i < RELAY_PIN_COUNT; i++) {
    pinMode(RELAY_PINS[i], INPUT_PULLUP);
    pinMode(RELAY_PINS[i], OUTPUT);
  }

  Serial.println("Initializing input pin states");
  for (int i = 0; i < INPUT_PIN_COUNT; i++) {
    pinMode(INPUT_PINS[i], INPUT);
  }

  //Set default pin state leveraging the internal resistors
  pinMode(LED_DRIVER_PIN, INPUT);
  pinMode(LED_DRIVER_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  fillLEDSineLevelTable();

  serialCommunicator.setup();

  initInterruptTimers();

  initLoopTimers();

  laserIdleStart = millis();

  Serial.println("--- Setup Completed ---");
}

void initLoopTimers() {
  Serial.println("--- Initializing timers ---");

  timer.cancel();  //Cancel all timers

  //Offset timers a bit so no single loop() execution performed too much work all at once
  //"timer jitter"

  // report laser status ever 1 second(s)
  timer.every(1000, publishStatusMessage);
  //init the WiFi immeiately / don't wait for the timer to kick off the init
  timer.in(1500, initialInitWiFi);
  // validate wifi status and reinit if necessary every ~30 seconds
  timer.every(61000, initWiFi);
  // validate mqtt  status and reinit if necessary every ~5 seconds
  timer.every(5100, initMQTT);
  // evaluate laser idle stake and wake/sleep as necessary
  timer.every(1000, evaluateLaserIdleState);

  Serial.print("--- Timers initialized: ");
  Serial.print(timer.size());
  Serial.println(" ---");
}

void initInterruptTimers() {
  Timer3.initialize(5000);  //5ms
  Timer3.attachInterrupt(ledTick);
}

//TODO CHRIS - send message on input change?

// ********************************************************* MAIN LOOP *********************************************************
void loop() {
  readInputStates();
  evaluateState();

  timer.tick();  // tick the timer

  serialCommunicator.loop();
  if (serialCommunicator.messageReceived()) {
    processSerialInput(serialCommunicator.getMessage());
  }
}

bool initialInitWiFi(void*) {
  if (DEBUG) {
    Serial.println("--- Starting initialInitWiFi ---");
  }

  if (wifiInitInProgress) {
    Serial.println("Exiting initialInitWiFi as init is already in progress");
    return true;
  }

  WiFi.init(&Serial1);
  initWiFi(nullptr);

  if (DEBUG) {
    Serial.println("--- Initialization execution started ---");
  }

  return true;
}

bool initWiFi(void*) {
  if (wifiInitInProgress) {
    Serial.println("Exiting initWiFi as init is already in progress");
    return true;
  }

  if (DEBUG) {
    Serial.println("Checking WiFi status...");
  }

  wifiStatus = WiFi.status();
  if (wifiStatus == WL_CONNECTED) {
    if (DEBUG) {
      Serial.println("WiFi still connected!");
    }
    return true;
  }

  if (wifiStatus == WL_NO_SHIELD) {
    Serial.println("WiFi module not detected");
    return true;
  }

  Serial.print("WiFi not connected. Attempting to connect WiFi to SSID: ");
  Serial.println(WIFI_SSID);

  wifiStatus = WiFi.begin(WIFI_SSID, WIFI_PASS);
  wifiInitInProgress = true;

  Serial.println("Starting WiFi checkWifiInitStatus timer...");
  timer.every(500, checkWifiInitStatus);

  return true;
}

//TODO add max retries or retry wait time
bool checkWifiInitStatus(void*) {
  Serial.println("Checking WiFi initialization status...");
  wifiStatus = WiFi.status();
  if (wifiStatus == WL_CONNECTED) {
    Serial.println("WiFi connected!");
    wifiInitInProgress = false;
    Serial.println();
    printCurrentNet();
    printWifiData();
    Serial.println();
    return false;  //Stop timer
  }

  return true;  //Keep the timer going
}

void printWifiData() {
  // *** print your WiFi shield's IP address
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  byte mac[6];
  WiFi.macAddress(mac);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", mac[5], mac[4], mac[3], mac[2], mac[1], mac[0]);
  Serial.print("MAC address: ");
  Serial.println(buf);
}

// Print WiFi connection details to serial Monitor  --------------------------------------------------------
void printCurrentNet() {
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // *** print the MAC address of the router you're attached to
  byte bssid[6];
  WiFi.BSSID(bssid);
  char buf[20];
  sprintf(buf, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[5], bssid[4], bssid[3], bssid[2], bssid[1], bssid[0]);
  Serial.print("BSSID: ");
  Serial.println(buf);

  long rssi = WiFi.RSSI();
  Serial.print("Signal strength (RSSI): ");
  Serial.println(rssi);
}

bool initMQTT(void*) {
  if (wifiInitInProgress || mqttInitInProgress) {
    Serial.println("Exiting initialInitWiFi as init is already in progress");
    return true;
  }

  if (wifiStatus != WL_CONNECTED) {
    Serial.println("WiFi not connected, aborting MQTT connection attempt");
    return true;
  }

  if (mqttClient.connected()) {
    if (DEBUG) {
      Serial.println("MQTT still connected!");
    }
    return true;
  }

  Serial.print("MQTT not connected. A new connection to ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.print(MQTT_PORT);
  Serial.println(" will be attempted");

  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setBufferSize(2048);

  mqttInitInProgress = true;

  Serial.println("Starting MQTT attemptMQTTConnection timer...");
  timer.every(500, attemptMQTTConnection);

  return true;
}

//TODO add max retries or retry wait time
bool attemptMQTTConnection(void*) {
  Serial.print("Initializing MQTT connection...");
  if (mqttClient.connect(MQTT_CLIENT_NAME)) {
    Serial.println("MQTT connected!");
    mqttInitInProgress = false;
    publishLogMessage("Successfully started and connected to WiFi/MQTT");
    return false;  //Stop timer
  } else {
    Serial.print("MQTT connection failed, State=");
    Serial.println(mqttClient.state());
    return true;  //Keep the timer going
  }
}

void readInputStates() {
  INPUT_STATES[INPUT_LASER_KEY_IDX] = getInputPinState(INPUT_LASER_KEY);
  INPUT_STATES[INPUT_STATUS_IDX] = getInputPinState(INPUT_STATUS);
  INPUT_STATES[INPUT_WIND_IDX] = getInputPinState(INPUT_WIND);
  INPUT_STATES[INPUT_IDLE_IDX] = getInputPinState(INPUT_IDLE);
  INPUT_STATES[INPUT_ERROR_IDX] = getInputPinState(INPUT_ERROR);
  INPUT_STATES[INPUT_FLOW_SENSOR_IDX] = getInputPinState(INPUT_FLOW_SENSOR);
  INPUT_STATES[INPUT_CHILLER_SENSOR_IDX] = getInputPinState(INPUT_CHILLER_SENSOR);
  INPUT_STATES[INPUT_DOOR_SENSOR_IDX] = getInputPinState(INPUT_DOOR_SENSOR);
}

void evaluateState() {
  const long now = millis();

  bool chillerFlowSensorState = INPUT_STATES[INPUT_FLOW_SENSOR_IDX] == HIGH;
  bool chillerSensorState = INPUT_STATES[INPUT_CHILLER_SENSOR_IDX] == HIGH;
  bool doorClosedSensorState = INPUT_STATES[INPUT_DOOR_SENSOR_IDX] == HIGH;
  bool statusSensorState = INPUT_STATES[INPUT_STATUS_IDX] == HIGH;
  bool windSensorState = INPUT_STATES[INPUT_WIND_IDX] == HIGH;

  bool existingPrimaryRedDotState = isRelayEnabled(RELAY_RED_DOT_PRIMARY_IDX);
  bool existingSecondaryRedDotState = isRelayEnabled(RELAY_RED_DOT_SECONDARY_IDX);
  bool existingLaserPowerState = isRelayEnabled(RELAY_LASER_POWER_IDX);
  bool existingChillerState = isRelayEnabled(RELAY_CHILLER_IDX);
  bool existingAirPrimaryState = isRelayEnabled(RELAY_AIR_PRIMARY_IDX);
  bool existingAirLowState = isRelayEnabled(RELAY_AIR_LOW_IDX);
  bool existingAirHighState = isRelayEnabled(RELAY_AIR_HIGH_IDX);

  bool newPrimaryRedDotState = existingPrimaryRedDotState;
  bool newSecondaryRedDotState = existingSecondaryRedDotState;
  bool newLaserPowerState = existingLaserPowerState;
  bool newChillerState = existingChillerState;
  bool newAirPrimaryState = existingAirPrimaryState;
  bool newAirLowState = existingAirLowState;
  bool newAirHighState = existingAirHighState;

  newPrimaryRedDotState = !inLightSleep;
  newSecondaryRedDotState = !inLightSleep;

  newChillerState = !inLightSleep;  //TODO CHRIS add delay for turning on to limit in-rush current?

  //TODO CHRIS ADD chillerFlowSensorState - disable if not OK
  //TODO CHRIS ADD chillerSensorState - disable if not OK

  
  //TODO on any other noticable change, wakeLaser() ? Any way to wake it via lightburn?

  if (!statusSensorState && jobStartTime != -1) {
    jobEndTime = now;
    //TODO CHRIS make more useful
    publishMessage("Finished");
  }

  if (statusSensorState || windSensorState) {
    if (jobStartTime == -1) {
      jobStartTime = now;
      jobEndTime = -1;
      //TODO CHRIS make more useful
      publishMessage("Started");
    }
  } else {
    jobStartTime = -1;
  }

  //Laser power supply
  if (!inLightSleep && INPUT_STATES[INPUT_LASER_KEY_IDX] == HIGH) {
    newLaserPowerState = true;
  } else {
    newLaserPowerState = false;
  }

  //TODO CHRIS Depending on idle state, keep the air on for a while longer
  //Air - low
  if (!inLightSleep && (statusSensorState || windSensorState)) {
    newAirPrimaryState = true;
    newAirLowState = true;
  } else {
    const long airDelayMs = POST_JOB_AIR_DELAY * 1000;
    if (now >= (jobEndTime + airDelayMs)) {
      newAirPrimaryState = false;

      //Leave the low air open another second to drain the syste,
      if (now >= (jobEndTime + airDelayMs + 1000)) {
        newAirLowState = false;
      }
    }
  }

  //Air high
  if (!inLightSleep && windSensorState) {
    newAirHighState = true;
  } else {
    newAirHighState = false;
  }

  /* Apply new states */
  if (newPrimaryRedDotState != existingPrimaryRedDotState)
    setRelayState(RELAY_RED_DOT_PRIMARY_IDX, RELAY_RED_DOT_PRIMARY, newPrimaryRedDotState);
  if (newSecondaryRedDotState != existingSecondaryRedDotState)
    setRelayState(RELAY_RED_DOT_SECONDARY_IDX, RELAY_RED_DOT_SECONDARY, newSecondaryRedDotState);
  if (newLaserPowerState != existingLaserPowerState)
    setRelayState(RELAY_LASER_POWER_IDX, RELAY_LASER_POWER, newLaserPowerState);
  if (newChillerState != existingChillerState)
    setRelayState(RELAY_CHILLER_IDX, RELAY_CHILLER, newChillerState);
  if (newAirPrimaryState != existingAirPrimaryState)
    setRelayState(RELAY_AIR_PRIMARY_IDX, RELAY_AIR_PRIMARY, newAirPrimaryState);
  if (newAirHighState != existingAirHighState)
    setRelayState(RELAY_AIR_HIGH_IDX, RELAY_AIR_HIGH, newAirHighState);
  if (newAirLowState != existingAirLowState)
    setRelayState(RELAY_AIR_LOW_IDX, RELAY_AIR_LOW, newAirLowState);

  if (statusSensorState) {
    wakeLaser();  //Keeps laserIdleStart moving forward to indicate the laser is not idle
  }

  //Force awake (if needed) when a job is running or when we detect a door state change
  if (doorClosedSensorState != CURRENT_DOOR_CLOSED) {
    wakeLaser();
  }

  //Update door state
  CURRENT_DOOR_CLOSED = doorClosedSensorState;
}

int getJobElapsedTime() {
  if (jobStartTime == -1) {
    return 0;
  }

  return (millis() - jobStartTime) / 1000;
}

const char* getSleepModeStr() {
  if (inDeepSleep) {
    return SLEEP_MODE_DEEP;
  } else if (inLightSleep) {
    return SLEEP_MODE_LIGHT;
  } else {
    return SLEEP_MODE_AWAKE;
  }
}

int getInputPinState(int pinNumber) {
  return digitalRead(pinNumber);
}

const char* boolToBooleanStr(bool value) {
  if (value == HIGH) {
    return TRUE_STR;
  }
  return FALSE_STR;
}

const char* digitalValueToBooleanStr(int value) {
  if (value == HIGH) {
    return TRUE_STR;
  }
  return FALSE_STR;
}

bool isRelayEnabled(int arrayIndex) {
  return RELAY_STATES[arrayIndex] == LOW;
}

void setRelayState(int arrayIndex, int pinNumber, bool state) {
  unsigned long now = millis();
  unsigned long timeDelta = now - RELAY_LAST_SWITCH[arrayIndex];

  int relayDebounce;

  if (state) {
    relayDebounce = RELAY_ON_DEBOUNCE[arrayIndex];
  } else {
    relayDebounce = RELAY_OFF_DEBOUNCE[arrayIndex];
  }

  if (timeDelta < relayDebounce) {
    return;
  }

  RELAY_LAST_SWITCH[arrayIndex] = now;

  if (state) {
    if (RELAY_STATES[arrayIndex] == HIGH) {
      digitalWrite(pinNumber, LOW);
      RELAY_STATES[arrayIndex] = LOW;
    }
  } else {
    if (RELAY_STATES[arrayIndex] == LOW) {
      digitalWrite(pinNumber, HIGH);
      RELAY_STATES[arrayIndex] = HIGH;
    }
  }
}

char* generateStatusJSON() {
  JsonDocument doc;

  doc["seqNo"] = ++seqNo;
  doc["jobElapsedTime"] = getJobElapsedTime();
  doc["idleTime"] = getIdleTime();
  doc["jobStartTime"] = jobStartTime;
  doc["jobEndTime"] = jobEndTime;
  doc["sleepMode"] = getSleepModeStr();
  doc["lightsMode"] = CURRENT_LED_MODE;  //TODO CHRIS translate to string
  doc["lightsLevel"] = ledsFadeCurrentLevel;

  JsonObject inputs = doc["inputs"].to<JsonObject>();
  inputs["laserKey"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_LASER_KEY_IDX]);
  inputs["status"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_STATUS_IDX]);
  inputs["wind"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_WIND_IDX]);
  inputs["idle"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_IDLE_IDX]);
  inputs["error"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_ERROR_IDX]);
  inputs["flowSensor"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_FLOW_SENSOR_IDX]);
  inputs["chillerSensor"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_CHILLER_SENSOR_IDX]);
  inputs["doorClosed"] = digitalValueToBooleanStr(INPUT_STATES[INPUT_DOOR_SENSOR_IDX]);

  JsonObject relays = doc["relays"].to<JsonObject>();
  relays["primaryRedDot"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_RED_DOT_PRIMARY_IDX));
  relays["secondaryRedDot"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_RED_DOT_SECONDARY_IDX));
  relays["laser"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_LASER_POWER_IDX));
  relays["chiller"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_CHILLER_IDX));
  relays["airPrimary"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_AIR_PRIMARY_IDX));
  relays["airLow"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_AIR_LOW_IDX));
  relays["airHigh"] = digitalValueToBooleanStr(isRelayEnabled(RELAY_AIR_HIGH_IDX));

  serializeJson(doc, JSON_BUFFER, JSON_BUFFER_SIZE);

  if (DEBUG) {
    Serial.println(JSON_BUFFER);
  }

  return JSON_BUFFER;
}

template<typename StringType>
bool publishLogMessage(StringType logMsg) {
  JsonDocument doc;

  doc["message"] = logMsg;

  serializeJson(doc, JSON_BUFFER, JSON_BUFFER_SIZE);

  if (DEBUG) {
    Serial.println(JSON_BUFFER);
  }

  return JSON_BUFFER;
}

bool publishStatusMessage(void*) {
  char* msg = generateStatusJSON();
  publishMessage(msg);

  return true;  //Keep the timer going
}

bool publishMessage(char* msg) {
  if (wifiStatus != WL_CONNECTED) {
    if (DEBUG) {
      Serial.println("Skipping MQTT publish as WiFi client is not connected");
    }
    return false;
  }

  if (!mqttClient.connected()) {
    if (DEBUG) {
      Serial.println("Skipping MQTT publish as MQTT client is not connected");
    }
    return false;
  }

  long start;
  if (DEBUG) {
    Serial.println("Publishing MQTT message...");
    start = millis();
  }

  bool publishSucces = mqttClient.publish(MQTT_TOPIC, msg);

  if (DEBUG) {
    long elapsed = millis() - start;
    Serial.print("Publish success: ");
    Serial.print(boolToBooleanStr(publishSucces));
    Serial.print(", Publish elapsed: ");
    Serial.println(elapsed);
  }

  return true;
}

//TODO CHRIS add simulated sleep (set the time manually)
void processSerialInput(char* command) {
  if (strcmp(command, "help") == 0) {
    printHelpMenu();
  } else if (strcmp(command, "diagnostics") == 0) {
    Serial.println("Calling printDiagnosticsInfo...");
    printDiagnosticsInfo();
  } else if (strcmp(command, "resetlooptimers") == 0) {
    Serial.println("Calling initLoopTimers...");
    initLoopTimers();
  } else if (strcmp(command, "connectmqtt") == 0) {
    Serial.println("Calling initMQTT...");
    initMQTT(nullptr);
  } else if (strcmp(command, "connectwifi") == 0) {
    Serial.println("Calling initWiFi...");
    initWiFi(nullptr);
  } else if (strcmp(command, "reinitwifi") == 0) {
    Serial.println("Calling initialInitWiFi...");
    initialInitWiFi(nullptr);
  } else if (strcmp(command, "toggledebug") == 0) {
    DEBUG = !DEBUG;
    Serial.print("New DEBUG value: ");
    Serial.println(boolToBooleanStr(DEBUG));
  } else if (strcmp(command, "publishstatus") == 0) {
    Serial.println("Calling publishStatusMessage...");
    publishStatusMessage(nullptr);
  } else if (strcmp(command, "ledsoff") == 0) {
    Serial.println("Turning off LEDs...");
    NEW_LED_MODE = LED_MODE_OFF;
  } else if (strcmp(command, "ledsfull") == 0) {
    Serial.println("Turning on LEDs (Full)...");
    NEW_LED_MODE = LED_MODE_FULL;
  } else if (strcmp(command, "ledsdim") == 0) {
    Serial.println("Turning on LEDs (Dim)...");
    NEW_LED_MODE = LED_MODE_DIM;
  } else if (strcmp(command, "ledsflashfull") == 0) {
    Serial.println("Turning on LEDs (Flash then Full)...");
    NEW_LED_MODE = LED_MODE_QUICK_FLASH_THEN_FULL;
  } else if (strcmp(command, "ledsflashdim") == 0) {
    Serial.println("Turning on LEDs (Flash then Dim)...");
    NEW_LED_MODE = LED_MODE_SLOW_FLASH_THEN_DIM;
  } else if (strcmp(command, "ledsflashoff") == 0) {
    Serial.println("Turning on LEDs (Flash then Off)...");
    NEW_LED_MODE = LED_MODE_SLOW_FLASH_THEN_OFF;
  }
}

void printHelpMenu() {
  Serial.println();
  Serial.println("===== Help / Supported Commands =====");
  Serial.println();
  Serial.println("  diagnostics");
  Serial.println("  resetlooptimers");
  Serial.println("  connectmqtt");
  Serial.println("  connectwifi");
  Serial.println("  reinitwifi");
  Serial.println("  toggledebug");
  Serial.println("  publishstatus");
  Serial.println("  ledsoff");
  Serial.println("  ledsfull");
  Serial.println("  ledsdim");
  Serial.println("  ledsflashfull");
  Serial.println("  ledsflashdim");
  Serial.println();
  Serial.println("=====================================");
  Serial.println();
}

void printDiagnosticsInfo() {
  checkWifiInitStatus(nullptr);  //Update WiFi status

  Serial.println();
  Serial.println("====== Diagnostics Information ======");
  Serial.println();

  //TODO CHRIS add sleep levels

  Serial.print("  DEBUG flag: ");
  Serial.println(boolToBooleanStr(DEBUG));

  Serial.print("  WiFi connected: ");
  Serial.println(boolToBooleanStr(wifiStatus == WL_CONNECTED));

  Serial.print("  MQTT connected: ");
  Serial.println(boolToBooleanStr(mqttClient.connected()));

  Serial.print("  Active loop timers: ");
  Serial.println(timer.size());

  Serial.print("  Current LED mode: ");
  Serial.println(CURRENT_LED_MODE);

  Serial.print("  Current LED fade level: ");
  Serial.println(ledsFadeCurrentLevel);

  Serial.print("  Latest sequence number: ");
  Serial.println(seqNo);

  Serial.println();
  Serial.println("=====================================");
  Serial.println();
}

unsigned long getIdleTime() {
  if (laserIdleStart <= 0)
    return -1;

  return (millis() - laserIdleStart) / 1000;
}

bool evaluateLaserIdleState(void*) {
  if (laserIdleStart <= 0)
    return true;  //Keep the timer going in the event of a init race condition

  unsigned long idleTime = getIdleTime();

  if (!inLightSleep && (idleTime >= LIGHT_SLEEP_IDLE_TIME)) {
    publishLogMessage("Forcing light sleep");
    publishStatusMessage(nullptr);
    lightSleepLaser();
  }

  if (!inDeepSleep && (idleTime >= DEEP_SLEEP_IDLE_TIME)) {
    publishLogMessage("Forcing deep sleep");
    publishStatusMessage(nullptr);
    deepSleepLaser();
  }

  return true;
}

void lightSleepLaser() {
  inLightSleep = true;
  //NEW_LED_MODE = LED_MODE_SLOW_FLASH_THEN_DIM;

  //TODO CHRIS UNTIL WE GET THE POWER SUPPLY HUMMING UNDER CONTROL
  NEW_LED_MODE = LED_MODE_OFF;

  //TODO CHRIS DEBUG
  Serial.println("Light sleep");
}

void deepSleepLaser() {
  inDeepSleep = true;
  NEW_LED_MODE = LED_MODE_OFF;

  //TODO CHRIS DEBUG
  Serial.println("Deep sleep");
}

void wakeLaser() {
  laserIdleStart = millis();

  if (!inLightSleep && !inDeepSleep)
    return;

  //TODO CHRIS DEBUG
  Serial.println("Waking laser");
  publishLogMessage("Waking laser");

  NEW_LED_MODE = LED_MODE_FULL;

  inLightSleep = false;
  inDeepSleep = false;

  //TODO CHRIS DEBUG
  Serial.println("Woke laser");
}

void ledTick() {
  int currentMode;
  int newMode;
  int newLedsFadeCurrentLevel;

  noInterrupts();
  currentMode = CURRENT_LED_MODE;
  newMode = NEW_LED_MODE;

  if (newMode != LED_MODE_NA) {
    //Set new mode states
    switch (newMode) {
      case LED_MODE_FULL:
        ledsTargetConstantLevel = 0;
        ledsFadeChange = 1;
        break;
      case LED_MODE_DIM:
        ledsTargetConstantLevel = 210;
        ledsFadeChange = 1;
        break;
      case LED_MODE_OFF:
        ledsTargetConstantLevel = 255;
        ledsFadeChange = 5;
        break;
      case LED_MODE_QUICK_FLASH_THEN_FULL:
        ledsFadeChange = 5;
        ledDimCyclesRemaining = 10;
        break;
      case LED_MODE_SLOW_FLASH_THEN_DIM:
        ledsFadeChange = 1;
        ledDimCyclesRemaining = 5;
        break;
      case LED_MODE_SLOW_FLASH_THEN_OFF:
        ledsFadeChange = 1;
        ledDimCyclesRemaining = 5;
        break;
    }

    //Reset 'new' state
    currentMode = CURRENT_LED_MODE = newMode;
    newMode = NEW_LED_MODE = LED_MODE_NA;
  }

  if (currentMode == LED_MODE_QUICK_FLASH_THEN_FULL
      || currentMode == LED_MODE_SLOW_FLASH_THEN_DIM
      || currentMode == LED_MODE_SLOW_FLASH_THEN_OFF) {
    if (ledsFadeCurrentLevel <= 0 || ledsFadeCurrentLevel >= 255) {
      ledsFadeChange = -ledsFadeChange;

      ledDimCyclesRemaining -= 1;

      if (ledDimCyclesRemaining <= 0) {
        if (currentMode == LED_MODE_SLOW_FLASH_THEN_DIM) {
          NEW_LED_MODE = LED_MODE_DIM;
        } else if (currentMode == LED_MODE_SLOW_FLASH_THEN_OFF) {
          NEW_LED_MODE = LED_MODE_OFF;
        } else {
          NEW_LED_MODE = LED_MODE_FULL;
        }
      }
    }

    ledsFadeCurrentLevel += ledsFadeChange;
  } else {
    //fade towards the target
    if (ledsTargetConstantLevel < ledsFadeCurrentLevel) {
      ledsFadeCurrentLevel -= ledsFadeChange;
      ledsFadeCurrentLevel = max(ledsFadeCurrentLevel, ledsTargetConstantLevel);
    } else if (ledsTargetConstantLevel > ledsFadeCurrentLevel) {
      ledsFadeCurrentLevel += ledsFadeChange;
      ledsFadeCurrentLevel = min(ledsFadeCurrentLevel, ledsTargetConstantLevel);
    }
  }

  if (ledsFadeCurrentLevel < 0) {
    ledsFadeCurrentLevel = 0;
  } else if (ledsFadeCurrentLevel > 255) {
    ledsFadeCurrentLevel = 255;
  }

  newLedsFadeCurrentLevel = ledsFadeCurrentLevel;  //local var
  interrupts();

  //PWM output the result:
  byte newLevel = ledsLevelTable[newLedsFadeCurrentLevel];
  analogWrite(LED_DRIVER_PIN, newLevel);
  analogWrite(LED_BUILTIN, newLevel);
}

void fillLEDSineLevelTable() {
  // set the range of values:
  float maxValue = 255;

  // iterate over the array and calculate the right value for it:
  for (int l = 0; l <= maxValue; l++) {

    // map input to a 0-179 range:
    float angle = map(l, 0, maxValue, 0, 179);
    float lightLevel = (sin((angle * PI / 180) + PI / 2) + 1) * 127.5;
    ledsLevelTable[l] = lightLevel;
  }
}