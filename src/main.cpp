// T&H&P
// march 2024

#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <Updater.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <AHT20.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "LittleFS.h"
#include <ArduinoJson.h>
#include <index.h>
#include "favicon.h"

#define MAJOR_VERSION "1"
#define MINOR_VERSION "0"

#define DEBUG 0

#define ESP_12_LED 2
#define NodeMCU_LED 16
#define DS18B20 14
#define POWER_EN 12
#define ADC_EN 13
#define ADC_bat A0
#define S1_SWITCH 10

#define U_PART U_FS

#if DEBUG
#define D_SerialBegin(...) Serial.begin(__VA_ARGS__);
#define D_print(...) Serial.print(__VA_ARGS__)
#define D_printf(...) Serial.printf(__VA_ARGS__)
#define D_write(...) Serial.write(__VA_ARGS__)
#define D_println(...) Serial.println(__VA_ARGS__)
#else
#define D_SerialBegin(...)
#define D_print(...)
#define D_printf(...)
#define D_write(...)
#define D_println(...)
#endif

bool bCaptive_portal;
bool bRestart = false;
bool bWifiConnected = false;

char wifi_ssid[33];
char wifi_password[33];
char wifi_ip[16];
char wifi_gw[16];
char wifi_subnet[16];
char wifi_bssid[18];

char strDeviceName[15];
char strSamplingRate[5];

char strMqttBroker[251];
char strMqttPort[6];
char strMqttUsername[65];
char strMqttPassword[65];

char strTemperatureTopic[51];
char strHumidityTopic[51];
char strVoltageTopic[51];

char strHost[32];
char strSwVersion[6];

char strTemperature[5];
char strHumidity[5];
char strVoltage[5];

const char *PARAM_SSID = "ssid";
const char *PARAM_PASS = "password";
const char *PARAM_IP = "ip-address";
const char *PARAM_GW = "gateway";
const char *PARAM_SUBNET = "subnet";

const char *PARAM_BSSID = "bssid";
const char *PARAM_CHANNEL = "channel";

const char *PARAM_DEVICE_NAME = "device-name";
const char *PARAM_SAMPLING_RATE = "sampling-rate";
const char *PARAM_MQTT_BROKER = "mqtt-broker";
const char *PARAM_MQTT_PORT = "mqtt-port";
const char *PARAM_MQTT_USERNAME = "mqtt-username";
const char *PARAM_MQTT_PASSSWORD = "mqtt-password";
const char *PARAM_MQTT_TOPIC_T = "mqtt-topic-T";
const char *PARAM_MQTT_TOPIC_H = "mqtt-topic-H";
const char *PARAM_MQTT_TOPIC_V = "mqtt-topic-V";

byte counter;

int mqttPort;
int intSamplingRate;
int32_t channel;
double DoubleSamplingRate;

int adc_bat_value = 0;
float fTemperature;
float fHumidity;
float fVoltage;

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;

size_t content_len;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

AHT20 aht20;

IPAddress ip;
IPAddress gateway;
IPAddress subnet;
IPAddress dns(1, 1, 1, 1);

AsyncWebServer server(80);

AsyncWebSocket ws("/ws");

JsonDocument readings;
JsonDocument readings1;
JsonDocument readings2;
JsonDocument readings3;

// FUNCTIONS
/* ------------------------------------------------------------------------------- */

void checkMQTTStatus();
void initWebSocket();
void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);
void notifyClients(String sensorReadings);
void initFS();
String getSensorReadings();
String getWifiConfig();
String getMQTTConfig();
String getSettings();
String processor(const String &var);
void LoadWifiConfig();
void LoadSettings();
void LoadMQTTConfig();
void ip_to_int(char strIp[16], IPAddress &ip_);
void saveWifiSettings();
bool cmpChannelBSSID();
void readAdcVoltage();
void readAHT20();
void webInit();
void handleUpdate(AsyncWebServerRequest *request);
void handleDoUpdate(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final);
void printProgress(size_t prg, size_t sz);

/* ------------------------------------------------------------------------------- */

void setup()
{
  WiFi.mode(WIFI_OFF);

  pinMode(ESP_12_LED, OUTPUT);
  digitalWrite(ESP_12_LED, HIGH);

  pinMode(S1_SWITCH, INPUT_PULLUP);

  if (digitalRead(S1_SWITCH) == LOW)
  {
    digitalWrite(ESP_12_LED, LOW);
    bCaptive_portal = true;
  }

  pinMode(ADC_EN, OUTPUT);
  pinMode(DS18B20, INPUT);
  pinMode(NodeMCU_LED, OUTPUT);
  pinMode(POWER_EN, OUTPUT);

  digitalWrite(NodeMCU_LED, HIGH);
  digitalWrite(POWER_EN, HIGH);
  digitalWrite(ADC_EN, LOW);

  strcpy(strSwVersion, MAJOR_VERSION);
  strcat(strSwVersion, ".");
  strcat(strSwVersion, MINOR_VERSION);

  D_SerialBegin(115200);
  snprintf(strHost, 11, "T_H-%06X", ESP.getChipId());
  D_println("\n");
  D_println(strHost);
  D_print("Version: ");
  D_println(strSwVersion);

  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();

  D_printf("Flash real size: %u bytes\n", realSize);
  D_printf("Flash ide  size: %u bytes\n", ideSize);

  Wire.begin();

  if (aht20.begin() == false)
  {
    D_print("AHT20 not detected");
    while (1)
      ;
  }
  else
  {
    D_print("\nAHT20 acknowledged");
  }

  readAdcVoltage();
  readAHT20();

  initFS();
  LoadWifiConfig();
  LoadSettings();
  LoadMQTTConfig();

  char *bssidString = wifi_bssid;
  const uint8_t bssid[] = {
      static_cast<uint8_t>(strtol(bssidString, NULL, 16)),
      static_cast<uint8_t>(strtol(bssidString + 3, NULL, 16)),
      static_cast<uint8_t>(strtol(bssidString + 6, NULL, 16)),
      static_cast<uint8_t>(strtol(bssidString + 9, NULL, 16)),
      static_cast<uint8_t>(strtol(bssidString + 12, NULL, 16)),
      static_cast<uint8_t>(strtol(bssidString + 15, NULL, 16))};

  D_println("Contents of the bssid array:");
  for (size_t i = 0; i < sizeof(bssid); i++)
  {
    D_print(bssid[i], HEX);
    D_print(" ");
  }

  ip_to_int(wifi_ip, ip);
  ip_to_int(wifi_gw, gateway);
  ip_to_int(wifi_subnet, subnet);

  // WIFI connect
  WiFi.forceSleepWake();
  if (bCaptive_portal)
  {
    D_print("\nEnter captive portal");
    if (strDeviceName[0] != '\0')
    {
      strcat(strHost, "-");
      strcat(strHost, strDeviceName);
    }
    WiFi.mode(WIFI_AP_STA);
    WiFi.softAP(strHost);
    D_print("Start AP_STA");
  }
  else
  {
    WiFi.mode(WIFI_STA);
  }

  WiFi.persistent(false); // dont write SSID and password to flash every time Wifi.begin is called and save connectivity time
  WiFi.config(ip, gateway, subnet, dns);
  WiFi.begin(wifi_ssid, wifi_password, channel, bssid);
  D_print("\nConnecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    D_print(".");
    delay(30);
    counter++;
    if (counter == 100)
    {
      D_println("No bssid and channel available");
      WiFi.begin(wifi_ssid, wifi_password);
    }
    if (counter >= 255)
    {
      D_println("No connection");
      break;
    }
    D_print(counter);
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    D_println("Connected");
    bWifiConnected = true;
    if (cmpChannelBSSID())
    {
      D_println("Save wifi settings");
      saveWifiSettings();
    }
  }
  else if (bCaptive_portal)
  {
    WiFi.mode(WIFI_AP);
    WiFi.softAP(strHost);
    D_print("Start AP");
  }
  else
  {
    D_print("No connection...restarting!");
    delay(200);
    ESP.restart();
  }

  D_print("\nConnected with IP: ");
  D_print(WiFi.localIP());
  D_printf("\nWiFi channel: %d", WiFi.channel());
  D_printf("\nWiFi BSSID: %s", WiFi.BSSIDstr().c_str());

  D_print("\nBroker:");
  D_println(strMqttBroker);
  D_print("\nport:");
  D_println(mqttPort);

  if (!bCaptive_portal && bWifiConnected)
  {
    mqttClient.setServer(strMqttBroker, mqttPort);
    while (!mqttClient.connected())
    {
      checkMQTTStatus();
    }

    if (mqttClient.publish(strTemperatureTopic, strTemperature, true))
    {
      D_print("\nwrite OK");
    }

    if (mqttClient.publish(strHumidityTopic, strHumidity, true))
    {
      D_print("\nwrite OK");
    }

    if (mqttClient.publish(strVoltageTopic, strVoltage, true))
    {
      D_print("\nwrite OK");
    }
    delay(200);
    mqttClient.disconnect();
    D_print("\nSLEEP");
    DoubleSamplingRate = intSamplingRate * 1E6;
    ESP.deepSleep(DoubleSamplingRate);
  }

  initWebSocket();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, PSTR("text/html"), MAIN_page, processor); });

  server.on("/wifi_save", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    String inputMessage;
    String inputParam;

    if (request->hasParam(PARAM_SSID, true))
    {
      inputMessage = request->getParam(PARAM_SSID, true)->value();
      inputMessage.toCharArray(wifi_ssid, inputMessage.length() + 1);
    }

    if (request->hasParam(PARAM_PASS, true))
    {
      inputMessage = request->getParam(PARAM_PASS, true)->value();
      inputMessage.toCharArray(wifi_password, inputMessage.length() + 1);
    }

    if (request->hasParam(PARAM_IP, true))
    {
      inputMessage = request->getParam(PARAM_IP, true)->value();
      inputMessage.toCharArray(wifi_ip, inputMessage.length() + 1);
    }

    if (request->hasParam(PARAM_GW, true))
    {
      inputMessage = request->getParam(PARAM_GW, true)->value();
      inputMessage.toCharArray(wifi_gw, inputMessage.length() + 1);
    }

    if (request->hasParam(PARAM_SUBNET, true))
    {
      inputMessage = request->getParam(PARAM_SUBNET, true)->value();
      inputMessage.toCharArray(wifi_subnet, inputMessage.length() + 1);
    }

    else
    {
      inputMessage = "No message sent";
      inputParam = "none";
    }
    saveWifiSettings();
    
    request->redirect("/"); });

  server.on("/settings_save", HTTP_POST, [](AsyncWebServerRequest *request)
            {

   if (request->hasParam(PARAM_DEVICE_NAME, true))
    {
      strcpy(strDeviceName, request->getParam(PARAM_DEVICE_NAME, true)->value().c_str());
    }
    readings3[PARAM_DEVICE_NAME] = strDeviceName;


   if (request->hasParam(PARAM_SAMPLING_RATE, true))
    {
      strcpy(strSamplingRate, request->getParam(PARAM_SAMPLING_RATE, true)->value().c_str());
    }
    intSamplingRate = atoi(strSamplingRate);
    if(intSamplingRate > 3600)
    {
      readings3[PARAM_SAMPLING_RATE] = "3600";
    }
    else if(intSamplingRate < 10)
    {
      readings3[PARAM_SAMPLING_RATE] = "10";
    }
    else
    {
      readings3[PARAM_SAMPLING_RATE] = strSamplingRate;
    }
   
    File configFile = LittleFS.open("/settings.json", "w");
    if (!configFile)
    {
      Serial.println("Failed to open config file for writing");
    }
    serializeJson(readings3, configFile);
    //configFile.println(jsonString);
    configFile.close();
    request->redirect("/"); });

  server.on("/MQTT_save", HTTP_POST, [](AsyncWebServerRequest *request)
            {

    if (request->hasParam(PARAM_MQTT_BROKER, true))
    {
      strcpy(strMqttBroker, request->getParam(PARAM_MQTT_BROKER, true)->value().c_str());
    }
    readings2[PARAM_MQTT_BROKER] = strMqttBroker;


    if (request->hasParam(PARAM_MQTT_PORT, true))
    {
      strcpy(strMqttPort, request->getParam(PARAM_MQTT_PORT, true)->value().c_str());
    }
    readings2[PARAM_MQTT_PORT] = strMqttPort;
    mqttPort = atoi(strMqttPort);


    if (request->hasParam(PARAM_MQTT_USERNAME, true))
    {
      strcpy(strMqttUsername, request->getParam(PARAM_MQTT_USERNAME, true)->value().c_str());
    }
    readings2[PARAM_MQTT_USERNAME] = strMqttUsername;
  

    if (request->hasParam(PARAM_MQTT_PASSSWORD, true))
    {
      strcpy(strMqttPassword, request->getParam(PARAM_MQTT_PASSSWORD, true)->value().c_str());
    }
    readings2[PARAM_MQTT_PASSSWORD] = strMqttPassword;


    if (request->hasParam(PARAM_MQTT_TOPIC_T, true))
    {
      strcpy(strTemperatureTopic, request->getParam(PARAM_MQTT_TOPIC_T, true)->value().c_str());
    }
    readings2[PARAM_MQTT_TOPIC_T] = strTemperatureTopic;


    if (request->hasParam(PARAM_MQTT_TOPIC_H, true))
    {
      strcpy(strHumidityTopic, request->getParam(PARAM_MQTT_TOPIC_H, true)->value().c_str());
    }
    readings2[PARAM_MQTT_TOPIC_H] = strHumidityTopic;


    if (request->hasParam(PARAM_MQTT_TOPIC_V, true))
    {
      strcpy(strVoltageTopic, request->getParam(PARAM_MQTT_TOPIC_V, true)->value().c_str());
    }
    readings2[PARAM_MQTT_TOPIC_V] = strVoltageTopic;

    File configFile = LittleFS.open("/MQTTConfig.json", "w");
    if (!configFile)
    {
      D_println("Failed to open config file for writing");
      readings2[PARAM_MQTT_BROKER] = "";
      readings2[PARAM_MQTT_PORT] = "";
      readings2[PARAM_MQTT_USERNAME] = "";
      readings2[PARAM_MQTT_PASSSWORD] = "";
      readings2[PARAM_MQTT_TOPIC_T] = "";
      readings2[PARAM_MQTT_TOPIC_H] = "";
      readings2[PARAM_MQTT_TOPIC_V] = "";
      
    }
    serializeJson(readings2, configFile);
    configFile.close();

    request->redirect("/"); });

  server.on("/restart", HTTP_POST, [](AsyncWebServerRequest *request)
            {
    D_println("Restart...");
    bRestart = true;
    request->redirect("/"); });

  server.on("/wifi", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/WifiConfig.json", "application/json"); });

  server.on("/MQTT", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send(LittleFS, "/MQTTConfig.json", "application/json"); });

  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "image/x-icon", favicon_ico, sizeof(favicon_ico)); });

  server.serveStatic("/", LittleFS, "/");

  if (!MDNS.begin("esp"))
  {
    D_println("Error starting mDNS");
    return;
  }
  MDNS.addService("http", "tcp", 80);
  webInit();

  server.begin();
}

/* ------------------------------------------------------------------------------- */

void loop()
{
  if ((millis() - lastTime) > timerDelay)
  {
    readAdcVoltage();
    readAHT20();
    String sensorReadings = getSensorReadings();
    notifyClients(sensorReadings);
    lastTime = millis();
    ws.cleanupClients();
  }

  if (bRestart == true)
  {
    delay(200);
    ESP.restart();
  }
}

/* ------------------------------------------------------------------------------- */

void checkMQTTStatus()
{
  while (!mqttClient.connected())
  {
    D_print("\nAttempting MQTT connection...");
    if (mqttClient.connect(strHost, strMqttUsername, strMqttPassword))
    {
      D_print("connected");
    }
    else
    {
      D_println(" MQTT connection failed, Error value=");
      D_print(mqttClient.state());
      D_println(" try again");
      delay(2000);
    }
  }
}

/* ------------------------------------------------------------------------------- */

String getSensorReadings()
{
  readings["temperature"] = strTemperature;
  readings["humidity"] = strHumidity;
  readings["voltage"] = strVoltage;
  // serializeJson(readings, Serial);
  String jsonString;
  serializeJson(readings, jsonString);
  return jsonString;
}

/* ------------------------------------------------------------------------------- */

void LoadWifiConfig()
{
  JsonDocument doc;
  D_println("Loading wifi settings");
  File file = LittleFS.open("/WifiConfig.json", "r");
  deserializeJson(doc, file);

  String ssid = doc[PARAM_SSID];
  ssid.toCharArray(wifi_ssid, ssid.length() + 1);

  String password = doc[PARAM_PASS];
  password.toCharArray(wifi_password, password.length() + 1);

  String ip = doc[PARAM_IP];
  ip.toCharArray(wifi_ip, ip.length() + 1);
  // Serial.println(wifi_ip);

  String gw = doc[PARAM_GW];
  gw.toCharArray(wifi_gw, gw.length() + 1);
  // Serial.println(wifi_gw);

  String subnet = doc[PARAM_SUBNET];
  subnet.toCharArray(wifi_subnet, subnet.length() + 1);
  // Serial.println(wifi_subnet);

  String bssid = doc[PARAM_BSSID];
  bssid.toCharArray(wifi_bssid, bssid.length() + 1);

  String channel_s = doc[PARAM_CHANNEL];
  channel = channel_s.toInt();

  file.close();
  // serializeJsonPretty(doc, Serial);
}

/* ------------------------------------------------------------------------------- */

void LoadSettings()
{
  String jsonString;
  JsonDocument doc;
  D_println("Loading settings");
  File file = LittleFS.open("/settings.json", "r");
  if (!file)
  {
    Serial.println("Failed to open config file for writing");
    doc[PARAM_DEVICE_NAME] = "";
    doc[PARAM_SAMPLING_RATE] = "";
    serializeJson(doc, file);
  }
  else
  {
    deserializeJson(doc, file);
    String device = doc[PARAM_DEVICE_NAME];
    device.toCharArray(strDeviceName, device.length() + 1);

    String sampling = doc[PARAM_SAMPLING_RATE];
    sampling.toCharArray(strSamplingRate, sampling.length() + 1);
    intSamplingRate = atoi(strSamplingRate);
    serializeJson(doc, jsonString);
  }
  file.close();
  // serializeJsonPretty(doc, Serial);
}

/* ------------------------------------------------------------------------------- */

void LoadMQTTConfig()
{
  JsonDocument doc;
  D_println("Loading MQTT settings");
  File file = LittleFS.open("/MQTTConfig.json", "r");
  deserializeJson(doc, file);

  String mqttBroker = doc[PARAM_MQTT_BROKER];
  mqttBroker.toCharArray(strMqttBroker, mqttBroker.length() + 1);

  String SmqttPort = doc[PARAM_MQTT_PORT];
  SmqttPort.toCharArray(strMqttPort, SmqttPort.length() + 1);
  mqttPort = atoi(strMqttPort);

  String mqttUser = doc[PARAM_MQTT_USERNAME];
  mqttUser.toCharArray(strMqttUsername, mqttUser.length() + 1);

  String mqttPassword = doc[PARAM_MQTT_PASSSWORD];
  mqttPassword.toCharArray(strMqttPassword, mqttPassword.length() + 1);

  String mqttTopicT = doc[PARAM_MQTT_TOPIC_T];
  mqttTopicT.toCharArray(strTemperatureTopic, mqttTopicT.length() + 1);

  String mqttTopicH = doc[PARAM_MQTT_TOPIC_H];
  mqttTopicH.toCharArray(strHumidityTopic, mqttTopicH.length() + 1);

  String mqttTopicV = doc[PARAM_MQTT_TOPIC_V];
  mqttTopicV.toCharArray(strVoltageTopic, mqttTopicV.length() + 1);

  file.close();
  // serializeJsonPretty(doc, Serial);
}

/* ------------------------------------------------------------------------------- */

String getWifiConfig()
{
  String jsonString;
  JsonDocument doc;
  File file = LittleFS.open("/WifiConfig.json", "r");
  deserializeJson(doc, file);
  file.close();
  serializeJson(doc, jsonString);
  // serializeJsonPretty(doc, Serial);
  return jsonString;
}

/* ------------------------------------------------------------------------------- */

String getMQTTConfig()
{
  String jsonString;
  JsonDocument doc;
  File file = LittleFS.open("/MQTTConfig.json", "r");
  deserializeJson(doc, file);
  file.close();
  serializeJson(doc, jsonString);
  // serializeJsonPretty(doc, Serial);
  return jsonString;
}

/* ------------------------------------------------------------------------------- */

String getSettings()
{
  String jsonString;
  JsonDocument doc;
  File file = LittleFS.open("/settings.json", "r");
  deserializeJson(doc, file);
  file.close();
  serializeJson(doc, jsonString);
  // serializeJsonPretty(doc, Serial);
  return jsonString;
}

/* ------------------------------------------------------------------------------- */

void initFS()
{
  if (!LittleFS.begin())
  {
    D_println("An error has occurred while mounting LittleFS");
  }
  else
  {
    D_println("\nLittleFS mounted successfully");
  }
}

/* ------------------------------------------------------------------------------- */

void notifyClients(String sensorReadings)
{
  ws.textAll(sensorReadings);
}

/* ------------------------------------------------------------------------------- */

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
  {
    data[len] = 0;
    String message = (char *)data;

    if (strcmp((char *)data, "getReadings") == 0)
    {
      String sensorReadings = getSensorReadings();
      notifyClients(sensorReadings);
    }
    String WifiConfig = getWifiConfig();
    notifyClients(WifiConfig);

    String MQTTsettings = getMQTTConfig();
    notifyClients(MQTTsettings);

    String settings = getSettings();
    notifyClients(settings);
  }
}

/* ------------------------------------------------------------------------------- */

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
  switch (type)
  {
  case WS_EVT_CONNECT:
    D_printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
    break;
  case WS_EVT_DISCONNECT:
    D_printf("WebSocket client #%u disconnected\n", client->id());
    break;
  case WS_EVT_DATA:
    handleWebSocketMessage(arg, data, len);
    break;
  case WS_EVT_PONG:
  case WS_EVT_ERROR:
    break;
  }
}

/* ------------------------------------------------------------------------------- */

void initWebSocket()
{
  ws.onEvent(onEvent);
  server.addHandler(&ws);
}

/* ------------------------------------------------------------------------------- */

void ip_to_int(char strIp[16], IPAddress &ip_)
{
  const char s[2] = ".";
  char bufferip[16];
  char *token;
  int i = 0;

  memcpy(bufferip, strIp, strlen(strIp) + 1);
  token = strtok(bufferip, s);

  while (token != NULL)
  {
    ip_[i] = atoi(token);
    token = strtok(NULL, s);
    i++;
    if (i > 3)
    {
      break;
    }
  }
}

/* ------------------------------------------------------------------------------- */

void saveWifiSettings()
{
  readings1[PARAM_SSID] = wifi_ssid;
  readings1[PARAM_PASS] = wifi_password;
  readings1[PARAM_IP] = wifi_ip;
  readings1[PARAM_GW] = wifi_gw;
  readings1[PARAM_SUBNET] = wifi_subnet;
  readings1[PARAM_BSSID] = wifi_bssid;
  readings1[PARAM_CHANNEL] = channel;

  File configFile = LittleFS.open("/WifiConfig.json", "w");
  if (!configFile)
  {
    D_println("Failed to open config file for writing");
  }
  serializeJson(readings1, configFile);
  configFile.close();
}

/* ------------------------------------------------------------------------------- */

// Replaces placeholder in HTML
String processor(const String &var)
{
  if (var == "VERSION")
  {
    return String(strSwVersion);
  }
  return String();
}

/* ------------------------------------------------------------------------------- */

bool cmpChannelBSSID()
{
  int32_t new_channel = int32_t(WiFi.channel());
  char new_wifi_bssid[18];
  bool isDifferent = false;

  String new_bssid_string = WiFi.BSSIDstr().c_str();
  new_bssid_string.toCharArray(new_wifi_bssid, new_bssid_string.length() + 1);

  if (strcmp(new_wifi_bssid, wifi_bssid) != 0)
  {
    strcpy(wifi_bssid, new_wifi_bssid);
    D_println("Bssid not equal!");
    isDifferent = true;
  }

  if (channel != new_channel)
  {
    channel = new_channel;
    D_println("Channel not equal!");
    isDifferent = true;
  }
  if (isDifferent)
  {
    return 1;
  }
  else
  {
    return 0;
  }
}

/* ------------------------------------------------------------------------------- */

void readAdcVoltage()
{
  digitalWrite(ADC_EN, HIGH);
  adc_bat_value = analogRead(ADC_bat);
  D_print("\nADC: ");
  D_print(adc_bat_value);
  fVoltage = (1071.0 / 191000.0 * adc_bat_value) + (43.0 / 9550.0);
  sprintf(strVoltage, "%.1f", fVoltage);
  D_print("\nVoltage: ");
  D_print(strVoltage);
  D_println("V");
  if (fVoltage < 3.0)
  {
    D_print("\nTurning off. Low battery!");
    delay(200);
    ESP.deepSleep(0);
  }
  digitalWrite(ADC_EN, LOW);
}

/* ------------------------------------------------------------------------------- */

void readAHT20()
{

  if (aht20.available() == true) // necessary to call twice???
  {
  }
  delay(50);
  if (aht20.available() == true)
  {
    fTemperature = aht20.getTemperature();
    fHumidity = aht20.getHumidity();

    sprintf(strTemperature, "%.1f", fTemperature);
    sprintf(strHumidity, "%.1f", fHumidity);

    D_print("\nTemperature: ");
    D_print(strTemperature);
    D_print(" C\n");
    D_print("Humidity: ");
    D_print(strHumidity);
    D_print("% RH");
  }
}

/* ------------------------------------------------------------------------------- */

void webInit()
{
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->redirect("/update"); });
  server.on("/update", HTTP_GET, [](AsyncWebServerRequest *request)
            { handleUpdate(request); });
  server.on(
      "/doUpdate", HTTP_POST,
      [](AsyncWebServerRequest *request) {},
      [](AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data,
         size_t len, bool final)
      { handleDoUpdate(request, filename, index, data, len, final); });

  server.onNotFound([](AsyncWebServerRequest *request)
                    { request->send(404); });
}

/* ------------------------------------------------------------------------------- */

void handleUpdate(AsyncWebServerRequest *request)
{
  const char *html = "<form method='POST' action='/doUpdate' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  request->send(200, "text/html", html);
}

/* ------------------------------------------------------------------------------- */

void handleDoUpdate(AsyncWebServerRequest *request, const String &filename, size_t index, uint8_t *data, size_t len, bool final)
{
  if (!index)
  {
    Serial.println("Update");
    content_len = request->contentLength();
    // if filename includes spiffs, update the spiffs partition
    int cmd = (filename.indexOf("littlefs") > -1) ? U_PART : U_FLASH;

    Update.runAsync(true);
    if (!Update.begin(content_len, cmd))
    {

      Update.printError(Serial);
    }
  }

  if (Update.write(data, len) != len)
  {
    Update.printError(Serial);
  }
  else
  {
    D_printf("Progress: %d%%\n", (Update.progress() * 100) / Update.size());
  }

  if (final)
  {
    AsyncWebServerResponse *response = request->beginResponse(302, "text/plain", "Please wait while the device reboots");
    response->addHeader("Refresh", "20");
    response->addHeader("Location", "/");
    request->send(response);
    if (!Update.end(true))
    {
      Update.printError(Serial);
    }
    else
    {
      D_println("Update complete");
      Serial.flush();
      ESP.restart();
    }
  }
}

/* ------------------------------------------------------------------------------- */

void printProgress(size_t prg, size_t sz)
{
  D_printf("Progress: %d%%\n", (prg * 100) / content_len);
}
