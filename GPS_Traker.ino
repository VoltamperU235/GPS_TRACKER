#define TINY_GSM_MODEM_SIM800

#include "TinyGPS.h"
#include <TinyGsmClient.h>
#include <PubSubClient.h>

#define SerialMon SerialUSB
#define SerialAT Serial1

// Serial2 pin and pad definitions (in Arduino files Variant.h & Variant.cpp)
#define PIN_SERIAL2_RX       (34ul)               // Pin description number for PIO_SERCOM on D12
#define PIN_SERIAL2_TX       (36ul)               // Pin description number for PIO_SERCOM on D10
#define PAD_SERIAL2_TX       (UART_TX_PAD_2)      // SERCOM pad 2
#define PAD_SERIAL2_RX       (SERCOM_RX_PAD_3)    // SERCOM pad 3

// Instantiate the Serial2 class
Uart Serial2(&sercom1, PIN_SERIAL2_RX, PIN_SERIAL2_TX, PAD_SERIAL2_RX, PAD_SERIAL2_TX);
TinyGPS gps;


void SERCOM1_Handler()    // Interrupt handler for SERCOM1
{
  Serial2.IrqHandler();
}

// Your GPRS credentials
// Leave empty, if missing user or pass
const char apn[]  = "ba.amx";


// MQTT details
const char* broker = "35.238.14.64";
const char* user = "admin";
const char* password = "FBx_admin2012";

const char* topicLed = "GsmClientTest/led";
const char* topicInit = "GsmClientTest/init";
const char* topicLedStatus = "GsmClientTest/ledStatus";

long int task1;
long int task2;

String mqtt_out = "";


TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);

#define LED_PIN 13
int ledStatus = LOW;

long lastReconnectAttempt = 0;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(9, OUTPUT);

  digitalWrite(9, HIGH);
  delay(1100);
  digitalWrite(9, LOW);

  // Set console baud rate
  SerialMon.begin(115200);
  delay(10);

  // Set GSM module baud rate
  SerialAT.begin(115200);
  delay(3000);

  Serial2.begin(9600);
  delay(500);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem: ");
  SerialMon.println(modemInfo);

  // Unlock your SIM card with a PIN
  //modem.simUnlock("1234");

  SerialMon.print("Waiting for network...");
  if (!modem.waitForNetwork()) {
    SerialMon.println(" fail");
    while (true);
  }
  SerialMon.println(" OK");

  SerialMon.print("Connecting to ");
  SerialMon.print(apn);
  if (!modem.gprsConnect(apn, "", "")) {
    SerialMon.println(" fail");
    while (true);
  }
  SerialMon.println(" OK");

  // MQTT Broker setup
  mqtt.setServer(broker, 1883);
  mqtt.setCallback(mqttCallback);

  Serial.print("Simple TinyGPS library v. "); Serial.println(TinyGPS::library_version());
  Serial.println("by Mikal Hart");
  task1 = millis();
  task2 = millis();
}

boolean mqttConnect() {
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);

  // Connect to MQTT Broker
  boolean status = mqtt.connect("GPS_TRAKER_U235", user, password);

  // Or, if you want to authenticate MQTT:
  //boolean status = mqtt.connect("GsmClientName", "mqtt_user", "mqtt_pass");

  if (status == false) {
    SerialMon.println(" fail");
    return false;
  }
  SerialMon.println(" OK");
  //mqtt.publish(topicInit, "GsmClientTest started");
  //mqtt.subscribe(topicLed);
  return mqtt.connected();
}

void check_mqtt()
{
  if (!mqtt.connected()) {
    SerialMon.println("=== MQTT NOT CONNECTED ===");
    // Reconnect every 10 seconds
    unsigned long t = millis();
    if (t - lastReconnectAttempt > 10000L) {
      lastReconnectAttempt = t;
      if (mqttConnect()) {
        lastReconnectAttempt = 0;
      }
    }
    delay(100);
    return;
  }
}

void loop() {

  check_mqtt();

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;


  while (Serial2.available())
  {
    char c = Serial2.read();

    //SerialUSB.write(c); // uncomment this line if you want to see the GPS data flowing
    if (gps.encode(c)) // Did a new valid sentence come in?
      newData = true;
  }


  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
//    SerialUSB.println("\n\n");
//    SerialUSB.print("LAT=");
//    SerialUSB.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
//    SerialUSB.print(" LON=");
//    SerialUSB.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
//    SerialUSB.print(" SAT=");
//    SerialUSB.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
//    SerialUSB.print(" PREC=");
//    SerialUSB.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
//    SerialUSB.println("\n\n");
    mqtt_out = String(flat, 6) + "," + String(flon, 6);
    //mqtt_out = "hello";



  }


  if (millis() > (task1 + 10000))
  {
    SerialUSB.println("Task1 - Management");
    task1 = millis();
    mqtt.publish("/MQTT", "System is OK");
  }

  if (millis() > (task2 + 5000))
  {
    if (newData)
    {
      SerialUSB.println("Task2 - GPS " + mqtt_out);
      
      task2 = millis();
      mqtt.publish("/MQTT", mqtt_out.c_str());
      mqtt_out = "";
      Serial2.flush();
    }

  }

  mqtt.loop();
}

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.print("Message arrived [");
  SerialMon.print(topic);
  SerialMon.print("]: ");
  SerialMon.write(payload, len);
  SerialMon.println();

  // Only proceed if incoming message's topic matches
  if (String(topic) == topicLed) {
    ledStatus = !ledStatus;
    digitalWrite(LED_PIN, ledStatus);
    mqtt.publish(topicLedStatus, ledStatus ? "1" : "0");
  }
}
