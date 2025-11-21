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


PeltierDevice::PeltierDevice(BLEServer* server) : hapticMotor(HapticMotor(20,21)){

    if (!server) {
        throw std::runtime_error("BLEServer pointer is null!");
    }
    else {
        Adafruit_PWMServoDriver pwm(0x40);
        //TODO: MAKE IT SO THE NUMBER OF HEATERS (and therefore, characteristics) can be set programmatically
        heaters = {
            HeaterController(pwm, A2, 5, 4),
            HeaterController(pwm, A1, 3, 2),
            HeaterController(pwm, A0, 0, 1),
        };
        temperaturesUpdated = {false, false, false};
        hrmonitor.setEnabled(true);
        hrmonitor.start_sensor();
        //Create BLE Service
        ble_service = server->createService(SERVICE_ID);
        //Define HeartRate Characteristic
        AddCharacteristic(
            "Heart Rate",
            HRCHAR_ID,
            BLECharacteristic::PROPERTY_BROADCAST |
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_NOTIFY,
            [&, this](BLECharacteristic* a, bool w) {
                if (!w) {
                    //Serial.write("HR read!");
                    a->setValue(std::to_string(this->hrmonitor.getLatestHR()));
                    //a->setValue(std::to_string((int) uxTaskGetStackHighWaterMark(NULL)));
                }
            }
        );

        //Define TargetTemp Characteristic
        AddCharacteristic(
            "Target Temperature 1",
            TARGET_TEMP1_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE, //want to have acknowledge, but not sure if the unity library supports it
            [&, this](BLECharacteristic* a, bool w) {
                if (w) {
                    targetUpdated = true;
                }
                else {
                    a->setValue(std::to_string(this->heaters[0].getCurrentTarget()));
                }
            }
        );

        //Define TargetTemp Characteristic
        AddCharacteristic(
            "Target Temperature 2",
            TARGET_TEMP2_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE, //want to have acknowledge, but not sure if the unity library supports it
            [&, this](BLECharacteristic* a, bool w) {
                if (w) {
                    targetUpdated = true;
                }
                else {
                    a->setValue(std::to_string(this->heaters[1].getCurrentTarget()));
                }
            }
        );

        //Define TargetTemp Characteristic
        AddCharacteristic(
            "Target Temperature 3",
            TARGET_TEMP3_ID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE, //want to have acknowledge, but not sure if the unity library supports it
            [&, this](BLECharacteristic* a, bool w) {
                if (w) {
                    targetUpdated = true;
                }
                else {
                    a->setValue(std::to_string(this->heaters[2].getCurrentTarget()));
                }
            }
        );

        //Define Haptic Characteristic
        AddCharacteristic(
            "Haptic Pulse",
            HAPTIC_MOTOR_ID,
            BLECharacteristic::PROPERTY_WRITE, //want to have acknowledge, but not sure if the unity library supports it
            [&, this](BLECharacteristic* a, bool w) {
                if (w) {
                    hapticMotor.pulse(std::stoi(a->getValue()));
                }
            }
        );

        ble_service->start();
        for (auto&& heater : heaters) {
            heater.begin();
        }
    }
}

void PeltierDevice::Update() {
    //We have to update temperature here, as doing much of anything inside of BLE callbacks causes a crash on the esp32 c3 :(
    //SEE: https://github.com/espressif/arduino-esp32/issues/7428
    if (temperaturesUpdated[0]) {
        try {
            heaters[0].setTargetTemperature(std::stof(ble_service->getCharacteristic(TARGET_TEMP1_ID)->getValue()));
        }
        catch (std::invalid_argument& e) {
            Serial.write("Unable to convert value to float for target temp");
        }
        temperaturesUpdated[0] = false;
    }

    if (temperaturesUpdated[1]) {
        try {
            heaters[1].setTargetTemperature(std::stof(ble_service->getCharacteristic(TARGET_TEMP2_ID)->getValue()));
        }
        catch (std::invalid_argument& e) {
            Serial.write("Unable to convert value to float for target temp");
        }
        temperaturesUpdated[1] = false;
    }

    if (temperaturesUpdated[2]) {
        try {
            heaters[2].setTargetTemperature(std::stof(ble_service->getCharacteristic(TARGET_TEMP3_ID)->getValue()));
        }
        catch (std::invalid_argument& e) {
            Serial.write("Unable to convert value to float for target temp");
        }
        temperaturesUpdated[2] = false;
    }

    hapticMotor.update();
    hrmonitor.process();
    for (auto&& heater : heaters) {
        heater.update();
    }


}

PeltierDevice::~PeltierDevice() {
    //I don't trust the ESP32 library to destroy the descriptors we declare using "new"
    ble_service->stop();
    for (int i = 0; i < Characteristics.size(); i++) {
        delete Characteristics[i]->getDescriptorByUUID(DESCRIPTOR_ID);
        //same with callback classes
        delete callbacks[i];
    }
    hrmonitor.stop();
};