#include <WiFi.h>
#include <SocketIoClient.h>
#include <ArduinoJson.h>
#include <DHT.h>


// Potentiometer GPIO
#define Red_Input         33
#define Green_Input       32
#define Blue_Input        35

// LED GPIO
#define LED_Red           5
#define LED_Green         18
#define LED_Blue          19

// PWM Chanel (0 - 16)
#define PWM_Chane1_Red    0
#define PWM_Chane1_Green  1
#define PWM_Chane1_Blue   2

#define PWM_Resolution    8
#define PWM_Frequency     1000

// Type of sensor that detects temperature and humidity.
#define sensorType DHT11

// Temperature And Humidity Sensor GPIO
#define temperatureAndHumiditySensorPin  23

DHT temperatureAndHumiditySensor(temperatureAndHumiditySensorPin, sensorType);
int humidity;
int temperature;

int red_value   = 0;
int green_value = 0;
int blue_value  = 0;

// Stores the amount of time the microcontroller has been on
unsigned long TimeWorking;
unsigned long elapsedTimeOfLastExecutionSetValues;
#define lapseOfTimeSetValues 100

/////////////////////////////////////
////// USER DEFINED VARIABLES //////
///////////////////////////////////
/// WIFI Settings ///
const char* ssid     = "change to ssid";
const char* password = "change to password";

/// Socket.IO Settings ///
bool useSSL                 = true; // Use SSL Authentication
char host[]                 = "192.168.0.147"; // Socket.IO Server Address
int port                    = 8081; // Socket.IO Port Address
char sslHost[]              = "backend-sensor-app.herokuapp.com"; // Socket.IO Server Address with ssl certificate
int sslPort                 = 443; // Socket.IO Port Address with ssl certificate
char path[]                 = "/socket.io/?transport=websocket"; // Socket.IO Base Path
const char * sslFingerprint = "";  // SSL Certificate Fingerprint
bool useAuth                = false; // use Socket.IO Authentication
const char * serverUsername = "socketIOUsername";
const char * serverPassword = "socketIOPassword";

/// Pin Settings ///
int LEDPin    = 2;
int buttonPin = 0;


/////////////////////////////////////
////// ESP32 Socket.IO Client //////
///////////////////////////////////

SocketIoClient webSocket;
WiFiClient client;

bool LEDState = false;


void socket_Connected(const char * payload, size_t length) {
  Serial.println("Socket.IO Connected!");
}

void socket_statusCheck(const char * payload, size_t length) {
  char* message = "\"ON\"";
  if (!LEDState) {
    message = "\"OFF\"";
  }
  webSocket.emit("status", message);
}

void socket_event(const char * payload, size_t length) {
  Serial.print("got message: ");
  Serial.println(payload);
}

void socket_pushButton(const char * payload, size_t length) {
  LEDStateChange(!LEDState);
}

void LEDStateChange(const bool newState) {
  char* message = "\"ON\"";
  if (!newState) {
    message = "\"OFF\"";
  }
  webSocket.emit("state_change", message);
  LEDState = newState;
  Serial.print("LED state has changed: ");
  Serial.println(message);
}

void checkLEDState() {
  digitalWrite(LEDPin, LEDState);
  const bool newState = digitalRead(buttonPin); // See if button is physically pushed
  if (!newState) {
    LEDStateChange(!LEDState);
    delay(250);
  }
}

void setPwmOutputs() {
  ledcAttachPin(LED_Red,    PWM_Chane1_Red);
  ledcAttachPin(LED_Green,  PWM_Chane1_Green);
  ledcAttachPin(LED_Blue,   PWM_Chane1_Blue);

  ledcSetup(PWM_Chane1_Red,   PWM_Frequency, PWM_Resolution);
  ledcSetup(PWM_Chane1_Green, PWM_Frequency, PWM_Resolution);
  ledcSetup(PWM_Chane1_Blue,  PWM_Frequency, PWM_Resolution);
}

void setPinStatus() {
  pinMode(LEDPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  setPwmOutputs();
}

void initializeWifiConnection() {
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setupOnListenEvents() {
  webSocket.on("connect", socket_Connected);
  webSocket.on("event", socket_event);
  webSocket.on("status", socket_statusCheck);
  webSocket.on("state_change_request", socket_pushButton);
}

void handleAuthentication() {
  if (useAuth) {
    webSocket.setAuthorization(serverUsername, serverPassword);
  }
}

void setupWebsocketConnection() {
  if (useSSL) {
    webSocket.beginSSL(sslHost, sslPort, path, sslFingerprint);
  } else {
    webSocket.begin(host, port, path);
  }

  handleAuthentication();
}

void setup() {

  Serial.begin(115200);
  while (!Serial) continue;

  setPinStatus();

  initializeWifiConnection();

  setupOnListenEvents();

  setupWebsocketConnection();

  temperatureAndHumiditySensor.begin();
}

String valuesToJson() {
  StaticJsonDocument<1024> jsonValues;
  JsonObject rgbValues = jsonValues.createNestedObject("rgbValues");
  rgbValues["red"] = red_value;
  rgbValues["green"] = green_value;
  rgbValues["blue"] = blue_value;
  
  JsonObject temperatureAndHumidityValues = jsonValues.createNestedObject("temperatureAndHumidityValues");
  temperatureAndHumidityValues["temperature"] = temperature;
  temperatureAndHumidityValues["humidity"] = humidity;
  
  String stringValues;

  serializeJson(jsonValues, stringValues);

  return stringValues;
}

bool isTimeToSetAndEmitValues() {
  TimeWorking = millis();
  if (TimeWorking - elapsedTimeOfLastExecutionSetValues >= lapseOfTimeSetValues) {
    elapsedTimeOfLastExecutionSetValues = TimeWorking;
    return true;
  } else {
    return false;
  }
}

void setPwmValues() {
  red_value   = map(analogRead(Red_Input),    0, 4095, 0, 255);
  green_value = map(analogRead(Green_Input),  0, 4095, 0, 255);
  blue_value  = map(analogRead(Blue_Input),   0, 4095, 0, 255);

  ledcWrite(PWM_Chane1_Red,   red_value);
  ledcWrite(PWM_Chane1_Green, green_value);
  ledcWrite(PWM_Chane1_Blue,  blue_value);
}

void setTemperatureAndHumidity() {
  humidity = temperatureAndHumiditySensor.readHumidity();
  temperature = temperatureAndHumiditySensor.readTemperature();
}

void emitValuesViaWebsocket() {
  String jsonvalues = valuesToJson();
  const char* message = const_cast<char*>(jsonvalues.c_str());
  webSocket.emit("values", message);
}

void setValues() {
  setPwmValues();
  setTemperatureAndHumidity();
}

void loop() {
  webSocket.loop();
  checkLEDState();
  if (isTimeToSetAndEmitValues()) {
    setValues();
    emitValuesViaWebsocket();
  }
}
