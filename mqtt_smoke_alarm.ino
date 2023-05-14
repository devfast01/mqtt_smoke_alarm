#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WiFiMulti.h>   // Include the Wi-Fi-Multi library
#include <ESP8266HTTPClient.h>
#include <ESP8266WebServer.h>

ESP8266WiFiMulti wifiMulti;     // Create an instance of the ESP8266WiFiMulti class, called 'wifiMulti'
ESP8266WebServer server(80);
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

const char* mqttServer   = "192.168.43.1";
const int   mqttPort     = 1883;
const char* mqttUser     = "";
const char* mqttPassword = "";

#define PUB_GPIO_STATUS "scale"
#define SUB_GPIO_ACTION "action"

String received_payload, ipAddress;
int relay_state;
unsigned long previousMillis = 0;
unsigned long interval = 30000;

int smoke = 5;
int fire = 16;
//LDR sensor pins
const int led = 2;

void setup() {
  Serial.begin(115200);
  pinMode(smoke, INPUT);
  pinMode(fire,INPUT);
  pinMode(led, OUTPUT);
  digitalWrite(led, HIGH);

  initWifiStation();
  initMQTTClient();

  server.on("/version/", catchVersion);
}

void loop() {
  mqttClient.loop();
  server.handleClient();

  unsigned long currentMillis = millis();
  // if WiFi is down, try reconnecting every CHECK_WIFI_TIME seconds
  if ((WiFi.status() != WL_CONNECTED) && (currentMillis - previousMillis >= interval)) {
    Serial.print(millis());
    Serial.println("Reconnecting to WiFi...");
    WiFi.disconnect();
    initWifiStation();
    previousMillis = currentMillis;
  }

int  smoke_val = digitalRead(smoke);
int  fire_val = digitalRead(fire);

  Serial.println(smoke_val);
  Serial.println(fire_val);
 
  if (smoke_val == 0) {
    digitalWrite(led, LOW);
    Serial.println("Smoke");
    mqttClient.publish(PUB_GPIO_STATUS, "0");
  }
  else if (fire_val == 1) {
    digitalWrite(led, LOW);
    Serial.println("Fire");
    mqttClient.publish(PUB_GPIO_STATUS, "1");
  }else {
        digitalWrite(led, HIGH);

  }

  delay(700);

}

//############### GETTING DATA FROM DEVICE WITH LOCALHOST #####################//
void catchVersion() {
  server.send(200, "text/html", "catch it");
}

void PubSubCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strstr(topic, SUB_GPIO_ACTION))
  {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '1') {
      digitalWrite(led, HIGH);   // Turn the LED on (Note that LOW is the voltage level
      Serial.println("LED HIGH");
      mqttClient.publish(PUB_GPIO_STATUS, "0");
    } else {
      digitalWrite(led, LOW);  // Turn the LED off by making the voltage HIGH
      Serial.println("LED HIGH");
      mqttClient.publish(PUB_GPIO_STATUS, "1");
    }
  }
}

//############### INIT WIFI STATION #####################//
void initWifiStation() {
  delay(10);
  Serial.println('\n');
  wifiMulti.addAP("Electronic", "12345678");   // add Wi-Fi networks you want to connect to
  wifiMulti.addAP("Test", "12345678");
  Serial.println("Connecting ...");
  int i = 0;
  while (wifiMulti.run() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  WiFi.hostname("Ahmet");
  Serial.println('\n');
  Serial.print("Connected to ");
  Serial.println(WiFi.SSID());              // Tell us what network we're connected to
  Serial.print("IP address:\t");
  Serial.println(WiFi.localIP());           // Send the IP address of the ESP8266 to the computer
  ipAddress = WiFi.localIP().toString();
  server.begin();
  Serial.println("HTTP server started");
}

//############### INIT MQTT CLIENT #####################//
void initMQTTClient() {
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(PubSubCallback);
  while (!mqttClient.connected()) {
    Serial.println(String("Connecting to MQTT (") + mqttServer + ")...");


    if (mqttClient.connect("arduinoClient2", mqttUser, mqttPassword)) {
      Serial.println("MQTT client connected");
    } else {
      Serial.print("\nFailed with state ");
      // Serial.println(mqttClient.state());
      if (WiFi.status() != WL_CONNECTED) {
        initWifiStation();
      }
      delay(5000);
    }
  }
  // Declare Pub/Sub topics
  mqttClient.publish(PUB_GPIO_STATUS, "active");
  mqttClient.subscribe(SUB_GPIO_ACTION);
}
