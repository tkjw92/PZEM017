#pragma once

#include <SoftwareSerial.h>
#include <ModbusMaster.h>

#define SHUNT_100A 0x0000
#define SHUNT_50A 0x0001
#define SHUNT_200A 0x0002
#define SHUNT_300A 0x0003

class PZEM017
{
private:
    SoftwareSerial sfserial;
    ModbusMaster node;

    static void preTransmission();
    static void postTransmission();

    void changeAddress(uint16_t new_addr);
    void setShunt(uint8_t addr, uint16_t shunt_value);

    uint8_t de;
    uint8_t re;
    uint8_t rx;
    uint8_t tx;
    uint8_t slave;

public:
    PZEM017();
    static PZEM017 *instance;

    void operator()(uint8_t rx, uint8_t tx);
    void begin(uint8_t new_slave_addr, uint16_t shunt, uint8_t de, uint8_t re);
    void resetEnergy();

    float readVoltage();
    float readCurrent();
    float readPower();
    uint16_t readEnergy();
};
