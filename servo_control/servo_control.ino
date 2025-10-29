
// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the GPIO used to control RGB LEDs.
#define RGB_LED 23
#define NUMPIXELS 10

// set the max ID.
int MAX_ID = 20;

// modeSelected.
// set the SERIAL_FORWARDING as true to control the servos with USB.
bool SERIAL_FORWARDING = false;

// set the interval of the threading.
#define threadingInterval 600
#define clientInterval 10

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "TCP_COMMUNICATION.h"
#include "secrets.h"
#include "RGB_CTRL.h"


// Define SECRET_SSID and SECRET_PASS in separete file secrets.h
const char* STA_SSID = SECRET_SSID;
const char* STA_PWD = SECRET_PASS;

TCPServer* tcpServer;

void setup() {
  Serial.begin(115200);
  tcpServer = new TCPServer(STA_SSID, STA_PWD);
  servoInit();
  InitRGB();
  RGBcolor(0, 64, 255);
  delay(1000);
}

void loop() {
  tcpServer->communicationLoop();
}



