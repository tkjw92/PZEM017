#include "PZEM017.h"

// Inisialisasi static member
PZEM017 *PZEM017::instance = nullptr;

// ========== Constructor ==========

PZEM017::PZEM017()
    : serial(nullptr), dePinNum(0), rePinNum(0), slaveAddress(0), initialized(false)
{
}

// ========== Public Methods ==========

bool PZEM017::begin(Stream &serialPort, uint8_t slaveAddr, uint16_t shuntType, uint8_t dePin, uint8_t rePin)
{
    // Validasi parameter
    if (slaveAddr == 0 || slaveAddr > 247)
    {
        return false;
    }

    // Set instance untuk callback (CRITICAL FIX #1)
    instance = this;

    // Simpan konfigurasi
    this->serial = &serialPort;
    this->slaveAddress = slaveAddr;
    this->dePinNum = dePin;
    this->rePinNum = rePin;

    // Setup RS485 control pins
    pinMode(dePinNum, OUTPUT);
    pinMode(rePinNum, OUTPUT);

    // Set initial state ke RX mode
    digitalWrite(rePinNum, LOW);
    digitalWrite(dePinNum, LOW);

    // Setup Modbus dengan callbacks
    node.begin(slaveAddress, *serial);
    node.preTransmission(preTransmission);
    node.postTransmission(postTransmission);

    // Tunggu stabilisasi
    delay(100);

    // Set shunt value
    if (!setShunt(shuntType))
    {
        initialized = false;
        return false;
    }

    // Verifikasi koneksi dengan membaca voltage
    delay(100);
    if (!isConnected())
    {
        initialized = false;
        return false;
    }

    initialized = true;
    return true;
}

bool PZEM017::changeAddress(uint8_t newAddress)
{
    if (newAddress == 0 || newAddress > 247)
    {
        return false;
    }

    // Gunakan broadcast address untuk mengubah alamat
    node.begin(ADDR_BROADCAST, *serial);

    uint8_t result = node.writeSingleRegister(REG_ADDR_CHANGE, newAddress);

    if (result != node.ku8MBSuccess)
    {
        // Kembalikan ke alamat semula
        node.begin(slaveAddress, *serial);
        return false;
    }

    // Tunggu perangkat memproses perubahan
    delay(200);

    // Update alamat dan verifikasi
    slaveAddress = newAddress;
    node.begin(slaveAddress, *serial);

    // Verifikasi dengan mencoba membaca
    bool connected = isConnected();

    if (!connected)
    {
        // Rollback jika gagal (tidak bisa dilakukan jika device benar-benar berubah)
        return false;
    }

    return true;
}

bool PZEM017::setShunt(uint16_t shuntType)
{
    if (!initialized && instance != this)
    {
        return false;
    }

    // Validasi shunt type
    if (shuntType > SHUNT_300A)
    {
        return false;
    }

    uint8_t result = node.writeSingleRegister(REG_SHUNT, shuntType);

    if (result != node.ku8MBSuccess)
    {
        return false;
    }

    delay(100); // Tunggu perangkat memproses
    return true;
}

bool PZEM017::resetEnergy()
{
    if (!initialized)
    {
        return false;
    }

    // Buat frame manual untuk command custom 0x42
    uint8_t frame[4];
    frame[0] = slaveAddress;
    frame[1] = CMD_RESET_ENERGY;

    // Hitung CRC
    uint16_t crc = calculateCRC16(frame, 2);
    frame[2] = lowByte(crc);
    frame[3] = highByte(crc);

    // Kirim command
    preTransmission();

    serial->write(frame, 4);
    serial->flush();

    postTransmission();

    // Tunggu respon (reset biasanya butuh waktu)
    delay(500);

    // Verifikasi dengan membaca energy (harusnya 0 atau mendekati 0)
    float energy;
    if (readEnergy(energy))
    {
        return (energy < 0.001); // Toleransi kecil
    }

    return false;
}

bool PZEM017::readVoltage(float &value)
{
    if (!initialized)
    {
        return false;
    }

    if (!readRegistersWithRetry(REG_VOLTAGE, 1))
    {
        return false;
    }

    // PZEM017: 1 LSB = 0.01V
    value = node.getResponseBuffer(0) * 0.01f;
    return true;
}

bool PZEM017::readCurrent(float &value)
{
    if (!initialized)
    {
        return false;
    }

    if (!readRegistersWithRetry(REG_CURRENT, 1))
    {
        return false;
    }

    // PZEM017: 1 LSB = 0.01A
    value = node.getResponseBuffer(0) * 0.01f;
    return true;
}

bool PZEM017::readPower(float &value)
{
    if (!initialized)
    {
        return false;
    }

    if (!readRegistersWithRetry(REG_POWER_L, 2))
    {
        return false;
    }

    // Ambil 2 register dan gabungkan (FIX #3: Endianness)
    uint16_t lowWord = node.getResponseBuffer(0);  // Register 0x0002 (LSW)
    uint16_t highWord = node.getResponseBuffer(1); // Register 0x0003 (MSW)

    uint32_t powerRaw = combine32BitValue(lowWord, highWord);

    // PZEM017: 1 LSB = 0.1W
    value = powerRaw * 0.1f;
    return true;
}

bool PZEM017::readEnergy(float &value)
{
    if (!initialized)
    {
        return false;
    }

    if (!readRegistersWithRetry(REG_ENERGY_L, 2))
    {
        return false;
    }

    // Ambil 2 register dan gabungkan (FIX #3: Endianness)
    uint16_t lowWord = node.getResponseBuffer(0);  // Register 0x0004 (LSW)
    uint16_t highWord = node.getResponseBuffer(1); // Register 0x0005 (MSW)

    uint32_t energyRaw = combine32BitValue(lowWord, highWord);

    // PZEM017: 1 LSB = 1Wh, konversi ke kWh
    value = energyRaw * 0.001f;
    return true;
}

bool PZEM017::readAll(float &voltage, float &current, float &power, float &energy)
{
    // Baca semua parameter dengan satu kali komunikasi untuk efisiensi
    bool success = true;

    success &= readVoltage(voltage);
    success &= readCurrent(current);
    success &= readPower(power);
    success &= readEnergy(energy);

    return success;
}

bool PZEM017::isConnected()
{
    if (!initialized && instance != this)
    {
        return false;
    }

    // Coba baca register voltage sebagai test koneksi
    uint8_t result = node.readInputRegisters(REG_VOLTAGE, 1);
    return (result == node.ku8MBSuccess);
}

// ========== Private Methods ==========

void PZEM017::preTransmission()
{
    if (instance)
    {
        // Set RS485 ke TX mode
        digitalWrite(instance->rePinNum, HIGH);
        digitalWrite(instance->dePinNum, HIGH);
        delayMicroseconds(RS485_DELAY_US);
    }
}

void PZEM017::postTransmission()
{
    if (instance)
    {
        delayMicroseconds(RS485_DELAY_US);

        // Set RS485 ke RX mode
        digitalWrite(instance->rePinNum, LOW);
        digitalWrite(instance->dePinNum, LOW);
    }
}

uint16_t PZEM017::calculateCRC16(const uint8_t *buffer, size_t length)
{
    uint16_t crc = 0xFFFF;

    for (size_t pos = 0; pos < length; pos++)
    {
        crc ^= (uint16_t)buffer[pos];

        for (uint8_t i = 8; i != 0; i--)
        {
            if ((crc & 0x0001) != 0)
            {
                crc >>= 1;
                crc ^= 0xA001;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}

bool PZEM017::readRegistersWithRetry(uint16_t regAddress, uint16_t regCount, uint8_t retries)
{
    for (uint8_t attempt = 0; attempt < retries; attempt++)
    {
        uint8_t result = node.readInputRegisters(regAddress, regCount);

        if (result == node.ku8MBSuccess)
        {
            return true;
        }

        // Jika gagal, tunggu sebelum retry
        if (attempt < retries - 1)
        {
            delay(RETRY_DELAY_MS);
        }
    }

    return false;
}

uint32_t PZEM017::combine32BitValue(uint16_t lowWord, uint16_t highWord)
{
    // PZEM017 menggunakan Little Endian format:
    // Byte order: [LSB_low, MSB_low, LSB_high, MSB_high]
    // Register order: [REG_L, REG_H]
    return ((uint32_t)highWord << 16) | lowWord;
}