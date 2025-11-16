#include <Arduino.h>
#if defined(ESP32) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#include <WiFi.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#endif

// This example is for ESP8266 and ESP32

#include <WiFiClient.h>
#include <WiFiClientSecure.h>

#include "secureConfig.h"
#include <FirebaseClient.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
unsigned long ms = 0;
int count = 0;

int IN1 = 33;
int IN2 = 25;
int IN3 = 26;
int IN4 = 27;

int IN5 = 4;
int IN6 = 14;
int IN7 = 16; // RX2
int IN8 = 17; // TX2

// Include the AccelStepper Library
#include <AccelStepper.h>

// Define step constant
#define MotorInterfaceType 8

// Creates an instance
// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequence (INT 3 and INT 2 they are swapped!!!!)
AccelStepper myStepper(MotorInterfaceType, IN1, IN3, IN2, IN4);
AccelStepper cameraStepper(MotorInterfaceType, IN5, IN7, IN6, IN8);

const int leftButtonPin = 32;
const int rightButtonPin = 13; // 34 and 35 didnt work well for some reason
int leftButtonState = 0;
int rightButtonState = 0;
int distance = 500000;

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastTime = 0;
// Timer set to 10 minutes (600000)
//unsigned long timerDelay = 600000;
// Set timer to 5 seconds (5000)
unsigned long timerDelay = 5000;

String serverNameForPosting = "https://posttestserver.dev/p/giorqm64yp8a4a6m/post";
const char* serverName = "https://rahulwebsite-d6b05-default-rtdb.firebaseio.com/rotations.json";

WiFiClientSecure client;

String sensorReadings;
float sensorReadingsArr[3];
String httpGETRequest(const char* serverName);

void fetchData() {
  //Send an HTTP POST request every 10 minutes
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
              
      sensorReadings = httpGETRequest(serverName);
      Serial.println(sensorReadings);
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

String httpGETRequest(const char* serverName) {
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
    
  // Your Domain name with URL path or IP address with path
  http.begin(client, serverName);
  
  // If you need Node-RED/server authentication, insert user and password below
  //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
  
  // Send HTTP POST request
  int httpResponseCode = http.GET();
  
  String payload = "{}"; 
  
  if (httpResponseCode>0) {
    Serial.print("HTTP Response code: ");
    Serial.println(httpResponseCode);
    payload = http.getString();
  }
  else {
    Serial.print("Error code: ");
    Serial.println(httpResponseCode);
  }
  // Free resources
  http.end();

  return payload;
}

void sendDataToServer() {
  //Send an HTTP POST request every 10 minutes
  if ((millis() - lastTime) > timerDelay) {
    //Check WiFi connection status
    if(WiFi.status()== WL_CONNECTED){
      HTTPClient http;

      String serverPath = serverNameForPosting + "?temperature=24.37";
      
      // Your Domain name with URL path or IP address with path
      http.begin(client, serverPath.c_str());
      
      // If you need Node-RED/server authentication, insert user and password below
      //http.setAuthorization("REPLACE_WITH_SERVER_USERNAME", "REPLACE_WITH_SERVER_PASSWORD");
      
      // Send HTTP GET request
      int httpResponseCode = http.GET();
      
      if (httpResponseCode>0) {
        Serial.print("HTTP Response code: ");
        Serial.println(httpResponseCode);
        String payload = http.getString();
        Serial.println(payload);
      }
      else {
        Serial.print("Error code: ");
        Serial.println(httpResponseCode);
      }
      // Free resources
      http.end();
    }
    else {
      Serial.println("WiFi Disconnected");
    }
    lastTime = millis();
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  // set the maximum speed, acceleration factor,
  // initial speed and the target position
  myStepper.setMaxSpeed(200.0);
  myStepper.setAcceleration(200.0);
  myStepper.setSpeed(200);
  myStepper.moveTo(distance);

  cameraStepper.setMaxSpeed(100.0);
  cameraStepper.setAcceleration(50.0);
  cameraStepper.setSpeed(50);
  pinMode(leftButtonPin, INPUT_PULLUP);
  pinMode(rightButtonPin, INPUT_PULLUP);
}

bool leftPressed = true;
bool rightPressed = true;

void loop() {
  leftButtonState = digitalRead(leftButtonPin);
  rightButtonState = digitalRead(rightButtonPin);
  // Change direction once the motor reaches target position
  if (myStepper.distanceToGo() == 0){
    myStepper.moveTo(-myStepper.currentPosition());
    Serial.println("reverse");
  }

  if (leftButtonState == HIGH) {
    //Serial.println("button pressed");
  } else {
    //    Serial.println("left button pressed");
    // if its the left button that was pressed, and the left button was not the last button pressed, we can reverse the stepper motor.
    if (leftPressed == true) {
      //Serial.println("left pressed!");
      leftPressed = false; // logic is reversed because i didnt see the motors decel, ill fix it later maybe
      rightPressed = true;
      distance = distance * - 1;
      myStepper.moveTo(distance);
    }
    
  }

  if (rightButtonState == HIGH) {

  } else {
    //Serial.println("right button pressed");
    if (rightPressed == true) {
      //Serial.println("right pressed!");
      rightPressed = false;
      leftPressed = true;
      distance = distance * -1;
      myStepper.moveTo(distance);
    }
  }
  // Move the motor one step
  myStepper.run();
  // check camera stepper
  //sendDataToServer();
  fetchData();
}

/*void testLoop() {
   if (millis() - ms > 15000 || ms == 0)
    {
        ms = millis();

        FirebaseJson json;
        FirebaseJsonData result;

        WiFiClient client;

        json.add("name", "esp");
        json.set("data/arr/[0]", count + 1);
        json.set("data/arr/[1]", count + 10);
        json.set("data/arr/[2]", count + 100);

        Serial.print("Connecting to server...");

        if (client.connect("httpbin.org", 80))
        {
            Serial.println(" ok");
            Serial.println("Send POST request...");
            client.print("POST /anything HTTP/1.1\n");
            client.print("Host: httpbin.org\n");
            client.print("Content-Type: application/json\n");
            client.print("Connection: close\n");
            client.print("Content-Length: ");
            client.print(json.serializedBufferLength(true));
            client.print("\n\n");
            json.toString(client, true);

            Serial.print("Read response...");

            // Automatically parsing for response (w or w/o header) with chunk encoding supported.
            if (json.readFrom(client))
            {
                Serial.println();
                json.toString(Serial, true);
                Serial.println("\n\nComplete");
            }
            else
                Serial_Printf(" failed with http code: %d\n", json.responseCode());
        }
        else
            Serial.println(" failed\n");

        client.stop();

        count++;
    }
}
    */

void firebaseSetup() {
  
}