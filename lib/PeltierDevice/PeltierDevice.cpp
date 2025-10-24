#include "PeltierDevice.h"

BLEUUID PeltierDevice::AddCharacteristic(std::string name, const char* uuid, uint32_t properties, std::function<void(BLECharacteristic*, bool)> callback, std::string defaultValue="") {
    BLEDescriptor* descriptor = new BLEDescriptor(DESCRIPTOR_ID);
    descriptor->setValue(name);
    BLECharacteristic* new_char = ble_service->createCharacteristic(
        uuid,
        properties
    );
    new_char->addDescriptor(descriptor);
    new_char->setValue(defaultValue);
    Characteristics.emplace_back(new_char);
    CharacteristicCallback* new_cb = new CharacteristicCallback(callback);
    callbacks.emplace_back(new_cb);
    new_char->setCallbacks(new_cb);
    return new_char->getUUID();
}


PeltierDevice::PeltierDevice(BLEServer* server) : HC(HeaterController(PWM_I2C_ADDR, THERM_PIN, SLEEP_PIN)){
    if (!server) {
        throw std::runtime_error("BLEServer pointer is null!");
    }
    else {
        //Create BLE Service
        ble_service = server->createService(SERVICE_ID);
        //Define HeartRate Characteristic
        AddCharacteristic(
            "Heart Rate",
            HRCHAR_ID,
            BLECharacteristic::PROPERTY_BROADCAST |
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY,
            [](BLECharacteristic* a, bool w) {
                if (!w) {
                    Serial.write("HR read!");
                    a->setValue(std::to_string((int) uxTaskGetStackHighWaterMark(NULL)));
                }
            }
        );

        //Define DutyCycle Characteristic
        AddCharacteristic(
            "Duty Cycle",
            DUTYCYC_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY,
            [&, this](BLECharacteristic* a, bool w) {
                if (!w) {
                    a->setValue(std::to_string(this->HC.getDutyCycle()));
                }
            }
        );

        //Define TargetTemp Characteristic
        AddCharacteristic(
            "Target Temperature",
            TARGETTEMP_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE, //want to have acknowledge, but not sure if the unity library supports it
            [&, this](BLECharacteristic* a, bool w) {
                if (w) {
                    targetUpdated = true;
                }
            },
            "30.0"
        );

        //Define RealTemp Characteristic
        AddCharacteristic(
            "Current Temperature",
            REALTEMP_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY,
            [&, this](BLECharacteristic* a, bool w) {
                if (!w) {
                    a->setValue(std::to_string(this->HC.getCurrentTemperature()));

                }
            }
        );

        ble_service->start();
        HC.begin(SDA_PIN, SCL_PIN);
    }
}

void PeltierDevice::Update() {
    //We have to update temperature here, as doing much of anything inside of BLE callbacks causes a crash on the esp32 c3 :(
    //SEE: https://github.com/espressif/arduino-esp32/issues/7428
    if (targetUpdated) {
        try {
            HC.setTargetTemperature(std::stof(ble_service->getCharacteristic(TARGETTEMP_ID)->getValue()));
        }
        catch (std::invalid_argument& e) {
            Serial.write("Unable to convert value to float for target temp");
            targetUpdated = false;
        }

    }
    HC.update();
}

PeltierDevice::~PeltierDevice() {
    //I don't trust the ESP32 library to destroy the descriptors we declare using "new"
    ble_service->stop();
    for (int i = 0; i < Characteristics.size(); i++) {
        delete Characteristics[i]->getDescriptorByUUID(DESCRIPTOR_ID);
        //same with callback classes
        delete callbacks[i];
    }
};