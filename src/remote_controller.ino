#include "WiFiEsp.h"
#include "SoftwareSerial.h"

#define joyX  A0 
#define joyY  A1  
#define sw 7    

// 시리얼 설정
SoftwareSerial espSerial(3, 2); // TX, RX 
WiFiEspClient client;

char ssid[] = "facedown";       
char pass[] = "dlsgur7053";  

void setup() {
  pinMode(sw, INPUT_PULLUP);
  Serial.begin(115200);     
  espSerial.begin(9600);      

  WiFi.init(&espSerial);

  // Wi-Fi 연결
  while (WiFi.status() != WL_CONNECTED) {
    Serial.println("Connecting to Wi-Fi...");
    WiFi.begin(ssid, pass);
    delay(5000);  
  }
  Serial.println("Connected to Wi-Fi");

  if (client.connect("192.168.0.11", 5000)) {
    Serial.println("Connected to Raspberry Pi");
  } else {
    Serial.println("Connection failed");
  }
}

void loop() {
  int joystickValueX = analogRead(joyX); 
  int joystickValueY = analogRead(joyY);

  int buttonState = digitalRead(sw);

  int mappedX = map(joystickValueX, 0, 1024, -100, 100);
  int mappedY = map(joystickValueY, 0, 1024, -100, 100);

  if (client.connected()) {
    String message = "X: " + String(mappedX) + " Y: " + String(mappedY) + " SW: " + String(buttonState);
    client.println(message);
    Serial.println("Sent: " + message);
  } else {
    Serial.println("Client not connected");
  }

  delay(100); 
}