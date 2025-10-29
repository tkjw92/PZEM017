#include <PZEM017.h>

PZEM017 *PZEM017::instance = nullptr;
PZEM017::PZEM017() {};
void PZEM017::operator()(uint8_t rx, uint8_t tx)
{
    this->rx = rx;
    this->tx = tx;

    this->instance = this;
}

void PZEM017::begin(uint8_t new_slave_addr, uint16_t shunt, uint8_t de, uint8_t re)
{
    this->de = de;
    this->re = re;
    this->slave = new_slave_addr;

    sfserial.begin(9600, SWSERIAL_8N1, rx, tx);

    pinMode(de, OUTPUT);
    pinMode(re, OUTPUT);

    postTransmission();

    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    changeAddress(slave);
    delay(200);
    setShunt(slave, shunt);
    delay(200);
}

void PZEM017::preTransmission()
{
    if (instance)
    {
        digitalWrite(instance->re, HIGH);
        digitalWrite(instance->de, HIGH);

        delayMicroseconds(500);
    }
}

void PZEM017::postTransmission()
{
    if (instance)
    {
        delayMicroseconds(500);

        digitalWrite(instance->re, LOW);
        digitalWrite(instance->de, LOW);
    }
}

void PZEM017::changeAddress(uint16_t new_addr)
{
    uint8_t general_address = 0xf8;
    uint16_t reg_addr = 0x0002;

    node.begin(general_address, sfserial);

    node.writeSingleRegister(reg_addr, new_addr);
}

void PZEM017::setShunt(uint8_t addr, uint16_t shunt_value)
{
    uint16_t reg_addr = 0x0003;
    node.begin(addr, sfserial);

    node.writeSingleRegister(reg_addr, shunt_value);
}

float PZEM017::readVoltage()
{
    uint8_t res = node.readInputRegisters(0x0000, 1);
    if (res == node.ku8MBSuccess)
    {
        return node.getResponseBuffer(0) * 0.01;
    }

    return 0;
}

float PZEM017::readCurrent()
{
    uint8_t res = node.readInputRegisters(0x0001, 1);
    if (res == node.ku8MBSuccess)
    {
        return node.getResponseBuffer(0) * 0.01;
    }

    return 0;
}

float PZEM017::readPower()
{
    uint8_t res = node.readInputRegisters(0x0002, 2);
    if (res == node.ku8MBSuccess)
    {
        return (((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0)) * 0.1;
    }

    return 0;
}

float PZEM017::readEnergy()
{
    uint8_t res = node.readInputRegisters(0x0004, 2);
    if (res == node.ku8MBSuccess)
    {
        uint32_t energy = ((uint32_t)node.getResponseBuffer(1) << 16) | node.getResponseBuffer(0);
        return energy * 0.001; // dalam kWh
    }
    return 0;
}

uint16_t modbus_crc16(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos];
        for (int i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
                crc >>= 1;
        }
    }
    return crc;
}

void PZEM017::resetEnergy()
{
    const uint16_t cmd = 0x42;
    uint8_t frame[4];

    frame[0] = slave;
    frame[1] = cmd;
    uint16_t crc = modbus_crc16(frame, 2);
    frame[2] = lowByte(crc);
    frame[3] = highByte(crc);

    preTransmission();
    sfserial.write(frame, 4);
    sfserial.flush();
    postTransmission();
}
