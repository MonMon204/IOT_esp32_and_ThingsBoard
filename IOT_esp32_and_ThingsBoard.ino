#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ModbusRTUSlave.h>

// ---------- Wi-Fi ----------
const char* ssid = "ASUENG Wi-Fi";
const char* password = "ASUeng_Demo11";

// ---------- ThingsBoard ----------
const char* mqttServer = "eu.thingsboard.cloud";
const int mqttPort = 1883;
const char* accessToken = "9VWoOoQpN5elO46trVDq";

#define SENDTOTHINGSBOARDTIME    1000

WiFiClient espClient;
PubSubClient client(espClient);

// ---------- RS-485 ----------
#define RS485_DE_RE 4
#define RS485_RX 16
#define RS485_TX 17

// ---------- LED ----------
#define LED_BUILTIN_PIN 2

// ---------- Modbus ----------
ModbusRTUSlave modbus(Serial2, RS485_DE_RE);
uint16_t holdingRegs[10];     // 40001–40010
bool discreteInputs[10];      // 10001–10010  (for PLC to read)

// ---------- Timers ----------
unsigned long lastSendTime = 0;

// ---------- MQTT Callback ----------
void callback(char* topic, byte* payload, unsigned int length) {
  payload[length] = '\0';
  String jsonStr = String((char*)payload);
  Serial.printf("Incoming MQTT: %s\n", jsonStr.c_str());

  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, jsonStr);
  if (error) {
    Serial.println("❌ JSON parse error!");
    return;
  }


  // send the received command from MQTT to PLC

  String method = doc["method"];
  int value = doc["params"];

  if (method == "setOutput") {
    bool state = (value == 1);

    // Set discrete input
    discreteInputs[0] = state;  // address 10001 → PLC reads at 0x10000
    digitalWrite(LED_BUILTIN_PIN, state ? HIGH : LOW);

    Serial.printf("✅ Discrete Input[0] set to %d (sent to PLC address 0x10000)\n", state);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN_PIN, OUTPUT);
  digitalWrite(LED_BUILTIN_PIN, LOW);

  // Start RS485 Serial
  Serial2.begin(9600, SERIAL_8N1, RS485_RX, RS485_TX);

  // Configure Modbus slave (ID = 4)
  modbus.begin(4, 9600, SERIAL_8N1);

  // Configure Modbus memory areas
  modbus.configureHoldingRegisters(holdingRegs, 10);
  modbus.configureDiscreteInputs(discreteInputs, 10);
  

  // Wi-Fi connection
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected!");

  // MQTT setup
  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  Serial.println("✅ Modbus RTU Slave ready (ID = 4)");
}


// ---------- MQTT Reconnect ----------
void reconnectMQTT() {
  while (!client.connected()) {
    Serial.print("Connecting to ThingsBoard...");
    if (client.connect("ESP32_Device", accessToken, NULL)) {
      Serial.println(" connected!");
      client.subscribe("v1/devices/me/rpc/request/+");
      Serial.println("✅ Subscribed to RPC commands");
    } else {
      Serial.print(" failed, rc=");
      Serial.println(client.state());
      delay(2000);
    }
  }
}


// ---------- Send Data to ThingsBoard ----------
void sendToThingsBoard(float temperature) {
  StaticJsonDocument<128> doc;
  doc["temperature"] = temperature;

  String payload;
  serializeJson(doc, payload);
  client.publish("v1/devices/me/telemetry", payload.c_str());
  Serial.println("📤 Sent to ThingsBoard: " + payload);
}


static float temperature = 0;
static float lastTemperature = 0;

// ---------- Loop ----------
void loop() {
  if (!client.connected()) reconnectMQTT();
  client.loop();

  // Poll for Modbus requests
  modbus.poll();

  // Read temperature written by PLC to holding register 40001 (index 0)
  temperature = holdingRegs[0];

  if (temperature != lastTemperature) {
    Serial.printf("🌡 New temperature received: %.1f °C \n", temperature);
    lastTemperature = temperature;
  }

  // Send to ThingsBoard every x seconds
  if (millis() - lastSendTime > SENDTOTHINGSBOARDTIME) {
    sendToThingsBoard(temperature);
    lastSendTime = millis();
  }
}
