#ifndef PELTIERDEVICE_H
#define PELTIERDEVICE_H

#include "HeaterController.h"
#include <BLEUtils.h>

// Define all the UUIDs
#define SERVICE_ID "86b56da5-ed87-46b0-8aea-1459aa1746ff"
#define HRCHAR_ID "21e1e876-b493-4495-a1fc-2da16c44072f"
#define DUTYCYC_ID "f678658e-e614-4a5b-b4ff-eba13df6ff4d"
#define TARGETTEMP_ID "14aaa533-6bd7-466b-9fd8-30bfdd203df9"
#define REALTEMP_ID "6d03b5ee-72eb-4d68-ab87-85cd84a699fe"
#define DESCRIPTOR_ID BLEUUID((uint16_t)0x2901)

//I2C Address
const uint8_t PWM_I2C_ADDR = 0x40;

//PINS
const uint8_t SDA_PIN = 6;
const uint8_t SCL_PIN = 7;
const int THERM_PIN = A0;
const int SLEEP_PIN = 10;

class CharacteristicCallback: public BLECharacteristicCallbacks {
    private:
    //Quick and dirty callback signature, gives the characteristic and a bool: true if it was a write, otherwise it was a read
    std::function<void(BLECharacteristic*, bool)> callback;

    public:
    CharacteristicCallback(std::function<void(BLECharacteristic*, bool)> callback) : callback(callback){
    };

    void onWrite(BLECharacteristic *pCharacteristic) {
        callback(pCharacteristic, true);
    }

    void onRead(BLECharacteristic *pCharacteristic) {
        callback(pCharacteristic, false);
    }
};

class PeltierDevice {
    private:
    HeaterController HC;
    BLEService* ble_service;
    std::vector<BLECharacteristic*> Characteristics;
    std::vector<CharacteristicCallback*> callbacks;
    bool targetUpdated = false;

    /// @brief Creates a new characteristic, adds it to our service, and returns the UUID for convenience
    /// @param name Name of the characteristic, used for the descriptor
    /// @param uuid The UUID of the Characteristic
    /// @param properties BLECharacteristic::PROPERTY_XXXX
    /// @param callback A std::function that takes a BLECharacteristic* and a bool. When the characteristic is written or read, it will call this function passing the characteristic as well as the bool which is true if the event was a write, or false if it was a read
    /// @param defaultValue The starting value to set the characteristic to
    /// @return UUID of the newly created characteristic, can be used in ble_server->getCharacteristic()
    BLEUUID AddCharacteristic(std::string name, const char* uuid, uint32_t properties, std::function<void(BLECharacteristic*, bool)> callback, std::string defaultValue);

    public:
    /**
     * @brief Initialize a new Peltier Device
     * @param server Pointer to the ESP32 BLEServer instance
     */
    PeltierDevice(BLEServer* server);

    /**
     * @brief Run main update loop for Peltier Device
     */
    void Update();

    /**
     * @brief Destroy the Peltier Device object and clean up resources
     */
    ~PeltierDevice();
};

#endif // PELTIERDEVICE_H