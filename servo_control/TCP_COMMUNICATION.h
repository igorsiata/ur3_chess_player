#include <WiFi.h>
#include "COMMAND_EXECUTOR.h"


class TCPServer {
public:
  TCPServer(const char* STA_SSID, const char* STA_PWD)
    : server(4210) {
    
    Serial.println("Trying to connect to wifi");
    connectToWiFi(STA_SSID, STA_PWD);
    printWiFiStatus();
  }

  void communicationLoop() {
    WiFiClient client = server.available();  // Check for incoming connections

    if (!client) return;

    Serial.println("Client connected.");
    while (client.connected()) {
      if (client.available()) {

        String command = readCommandFromClient(client);
        executeCommand(command, client);
      }
    }
    client.stop();
    Serial.println("Client disconnected.");
  }

private:
  WiFiServer server;
  CommandExecutor commandExecutor;
  const int IP[4] = {192, 168, 0, 120};

  void connectToWiFi(const char* STA_SSID, const char* STA_PWD) {   
    assignStaticIP();
    WiFi.begin(STA_SSID, STA_PWD);
    unsigned long startMillis = millis();
    while (WiFi.status() != WL_CONNECTED) {
      Serial.println("...");
      if (millis() - startMillis > 10000) {  // 10-second timeout
        Serial.println("Error: WiFi connection timed out!");
        return;
      }
      delay(500);
    }
    server.begin();
  }

  void assignStaticIP() {
    IPAddress localIP(IP[0], IP[1], IP[2], IP[3]);
    IPAddress gateway(192, 168, 0, 1);
    IPAddress subnet(255, 255, 255, 0);

    // Set the static IP
    WiFi.config(localIP, gateway, subnet);
  }

  void printWiFiStatus() {
    Serial.println("\nWiFi connected.");
    Serial.print("ESP32 IP: ");
    Serial.println(WiFi.localIP());
    Serial.println("TCP Server started, waiting for connections...");
  }

  String readCommandFromClient(WiFiClient client) {
    String command = client.readStringUntil('\n');
    command.trim();
    Serial.printf("Received: %s\n", command.c_str());
    return command;
  }

  void executeCommand(String command, WiFiClient client) {
    String response = commandExecutor.executeCommand(command);
    Serial.printf("Sending: %s\n", response.c_str());
    client.println(response);
  }
};
