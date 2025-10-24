#include "PeltierDevice.h"
#include <BLEDevice.h>
#include <BLEServer.h>

PeltierDevice* device;

class ConnectionManager : public BLEServerCallbacks {
    public:
    bool isConnected = false;

    void onConnect(BLEServer* server) override {
        BLEServerCallbacks::onConnect(server);
        Serial.println("Connected!");
        isConnected = true;
    }

    void onDisconnect(BLEServer* server) override {
        BLEServerCallbacks::onDisconnect(server);
        Serial.println("Disconnected!");
        isConnected = false;
        server->startAdvertising();
    }
};


void setup () {
  Serial.begin(115200);
  //Setup BLE
  BLEDevice::init("PeltierDevice");
  BLEServer* Server = BLEDevice::createServer();

  //This will need to last the lifetime of the program, so in this case it is safe to use new here even though arduino offers no "destructor" at the end of the program
  Server->setCallbacks(new ConnectionManager());

  //Again, arduino patterns mean that if we want to initialize in setup but use in loop, we have to use a pointer and initialize on the heap :(
  device = new PeltierDevice(Server);
  Server->startAdvertising();
}

void loop() {
    device->Update();
}
