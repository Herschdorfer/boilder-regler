#include "Adafruit_ST77xx.h"
#include "Client.h"
#include "ESP32_PWM.h"
#include "HardwareSerial.h"
#include "WiFi.h"
#include "esp32-hal.h"
#include "esp_task_wdt.h"    // Include the header file for ESP watchdog
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <ArduinoJson.h>
#include <PubSubClient.h> // Include the header file for PubSubClient library
#include <SPI.h>
#include <WiFiManager.h> // Include the header file for WiFiManager library
#include <cstdint>

#include "uptime.hpp"

// pinouts from https://github.com/Xinyuan-LilyGO/TTGO-T-Display
#define TFT_MOSI 19
#define TFT_SCLK 18
#define TFT_CS   5
#define TFT_DC   16
#define TFT_RST  23
#define TFT_BL   4

#define HW_TIMER_INTERVAL_US 20L

const char *const mqtt_server = "omv"; // mqtt server

const char *const topic_status = "/boiler/status";
const char *const topic_power_max = "/boiler/control";

WiFiClient espClient;
PubSubClient client(espClient); // lib required for mqtt

// constructor for data object named tft
Adafruit_ST7789 tft =
    Adafruit_ST7789(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

WiFiManager wifiManager;

Uptime uptime;
// Init ESP32_ISR_PWM
ESP32_PWM ISR_PWM;

// Init ESP32 timer 1
ESP32Timer ITimer(1);
uint32_t PWM_Pin = TFT_BL; // Pin 2 = ESP32 GPIO 2

uint16_t power_max = 0;
uint16_t power_desired = 0;
uint16_t power_actual = 0;

uint8_t channelNum;

float PWM_Freq1 = 100.0f;

// You can assign any duty_cycle for any PWM here, from 0-100
float PWM_DutyCycle1 = 50.0;

void parse_json(byte *payload, unsigned int length) {

  JsonDocument doc;
  deserializeJson(doc, payload, length);

  if (doc.containsKey("power_max")) {
    power_max = doc["power_max"];
  }

  if (doc.containsKey("power_desired")) {
    power_desired = doc["power_desired"];
  }

  if (doc.containsKey("power_actual")) {
    power_actual = doc["power_actual"];
  }
}

void callback(char const *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }

  if (strcmp(topic, topic_power_max) == 0) {
    parse_json(payload, length);
  }
}

void reconnect() {
  while (!client.connected()) {
    if (client.connect("ESP32_clientID")) {

      client.publish(topic_status, "Nodemcu connected to MQTT");

      client.subscribe(topic_power_max);

    } else {
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void connectMqtt() {
  client.connect("ESP32_clientID");
  client.publish(topic_status, "connected to MQTT");

  client.subscribe(topic_power_max);

  if (!client.connected()) {
    reconnect();
  }
}

bool IRAM_ATTR TimerHandler(void *) {
  ISR_PWM.run();

  return true;
}

[[noreturn]] void handleTft(void *) {
  for (;;) {
    tft.setTextWrap(false);
    tft.setCursor(0, 0);
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK);
    tft.setTextSize(1);

    tft.println("Connected to: ");
    tft.println(wifiManager.getWiFiSSID());
    tft.println("IP address: ");
    tft.println(WiFi.localIP());

    tft.print("Uptime: ");
    tft.println(uptime.getUptimeInSeconds());

    tft.print("Power max: ");
    tft.print(power_max);
    tft.println("    ");

    tft.print("Power des: ");
    tft.print(power_desired);
    tft.println("    ");

    tft.print("Power act: ");
    tft.print(power_actual);
    tft.println("    ");

    tft.print("Duty Cycle: ");
    tft.print(PWM_DutyCycle1);
    tft.println("%   ");

    delay(1000);
  }
}

[[noreturn]] void handleMqtt(void *) {
  for (;;) {
    if (!client.connected()) {
      reconnect();
    }

    client.loop();

    delay(100);
  }
}

void setup(void) {
  Serial.begin(9600);

  wifiManager.autoConnect("Boiler-Controller");

  client.setServer(mqtt_server, 1883); // connecting to mqtt server
  client.setCallback(callback);

  pinMode(TFT_BL, OUTPUT); // TTGO T-Display enable Backlight pin 4
  // digitalWrite(TFT_BL, HIGH); // T-Display turn on Backlight
  tft.init(135, 240); // Initialize ST7789 240x135
  tft.fillScreen(ST77XX_BLACK);

  connectMqtt();

  if (ITimer.attachInterruptInterval(HW_TIMER_INTERVAL_US, TimerHandler)) {
    Serial.print(F("Starting ITimer OK = "));
  } else
    Serial.println(F("Can't set ITimer. Select another freq. or timer"));

  channelNum = (uint8_t)ISR_PWM.setPWM(PWM_Pin, PWM_Freq1, PWM_DutyCycle1);

  xTaskCreate(handleTft, "Task_TFT", 2000, nullptr, 1, nullptr);

  xTaskCreate(handleMqtt, "Task_MQTT", 2000, nullptr, 2, nullptr);
}

void loop() {
  esp_task_wdt_init(1,
                    true); // Enable ESP watchdog with a timeout of 1 seconds

  if (!client.connected()) {
    reconnect();
  }

  if (power_desired > 0 && power_max > 0 && power_desired <= power_max) {
    PWM_DutyCycle1 =
        static_cast<float>(power_desired) / static_cast<float>(power_max) * 100;
  }

  Serial.print(F("Duty Cycle: "));
  Serial.println(PWM_DutyCycle1);

  if (!ISR_PWM.modifyPWMChannel(channelNum, PWM_Pin, PWM_Freq1,
                                PWM_DutyCycle1)) {
    Serial.print(F("modifyPWMChannel error for PWM_Period"));
  }
  delay(50);

  esp_task_wdt_reset();
}
