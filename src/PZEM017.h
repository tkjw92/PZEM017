#ifndef PZEM017_H
#define PZEM017_H

#include <Arduino.h>
#include <Stream.h>
#include <ModbusMaster.h>

// Definisi nilai shunt yang didukung
#define SHUNT_100A 0x0000
#define SHUNT_50A 0x0001
#define SHUNT_200A 0x0002
#define SHUNT_300A 0x0003

/**
 * @class PZEM017
 * @brief Driver untuk sensor daya PZEM017 dengan komunikasi Modbus RTU
 *
 * Kelas ini menyediakan interface untuk membaca parameter listrik dari
 * sensor PZEM017 melalui protokol Modbus RTU dengan RS485.
 *
 * @note Mendukung HardwareSerial dan SoftwareSerial
 * @author Refactored Version
 * @version 2.0
 */
class PZEM017
{
private:
    // ========== Register Addresses ==========
    static constexpr uint16_t REG_VOLTAGE = 0x0000;     // Tegangan (V)
    static constexpr uint16_t REG_CURRENT = 0x0001;     // Arus (A)
    static constexpr uint16_t REG_POWER_L = 0x0002;     // Daya LSW (W)
    static constexpr uint16_t REG_POWER_H = 0x0003;     // Daya MSW (W)
    static constexpr uint16_t REG_ENERGY_L = 0x0004;    // Energi LSW (Wh)
    static constexpr uint16_t REG_ENERGY_H = 0x0005;    // Energi MSW (Wh)
    static constexpr uint16_t REG_ADDR_CHANGE = 0x0002; // Register untuk ubah alamat
    static constexpr uint16_t REG_SHUNT = 0x0003;       // Register untuk set shunt

    // ========== Commands ==========
    static constexpr uint8_t CMD_RESET_ENERGY = 0x42; // Custom command reset energi
    static constexpr uint8_t ADDR_BROADCAST = 0xF8;   // Alamat broadcast

    // ========== Configuration ==========
    static constexpr uint8_t DEFAULT_RETRIES = 3;      // Jumlah retry default
    static constexpr uint16_t RETRY_DELAY_MS = 50;     // Delay antar retry
    static constexpr uint32_t MODBUS_BAUD_RATE = 9600; // Baud rate Modbus
    static constexpr uint16_t RS485_DELAY_US = 500;    // Delay switching RS485

    // ========== Private Members ==========
    Stream *serial;    // Pointer ke serial port (HW atau SW)
    ModbusMaster node; // Instance Modbus Master

    uint8_t dePinNum;     // Driver Enable pin (RS485)
    uint8_t rePinNum;     // Receiver Enable pin (RS485)
    uint8_t slaveAddress; // Alamat Modbus slave
    bool initialized;     // Flag status inisialisasi

    // Static instance untuk callback
    static PZEM017 *instance;

    // ========== Private Methods ==========

    /**
     * @brief Callback sebelum transmisi Modbus (set RS485 ke TX mode)
     */
    static void preTransmission();

    /**
     * @brief Callback setelah transmisi Modbus (set RS485 ke RX mode)
     */
    static void postTransmission();

    /**
     * @brief Menghitung CRC16 Modbus
     * @param buffer Data buffer
     * @param length Panjang data
     * @return Nilai CRC16
     */
    uint16_t calculateCRC16(const uint8_t *buffer, size_t length);

    /**
     * @brief Membaca register Modbus dengan retry
     * @param regAddress Alamat register
     * @param regCount Jumlah register
     * @param retries Jumlah percobaan
     * @return true jika berhasil
     */
    bool readRegistersWithRetry(uint16_t regAddress, uint16_t regCount, uint8_t retries = DEFAULT_RETRIES);

    /**
     * @brief Menggabungkan 2 register 16-bit menjadi 32-bit
     * @param lowWord Register LSW (Low Word)
     * @param highWord Register MSW (High Word)
     * @return Nilai 32-bit
     * @note PZEM017 menggunakan format Little Endian (LSW first)
     */
    uint32_t combine32BitValue(uint16_t lowWord, uint16_t highWord);

public:
    /**
     * @brief Constructor
     */
    PZEM017();

    /**
     * @brief Inisialisasi komunikasi dengan PZEM017
     * @param serialPort Reference ke Stream (HardwareSerial atau SoftwareSerial)
     * @param slaveAddr Alamat Modbus slave (1-247)
     * @param shuntType Tipe shunt (SHUNT_100A, SHUNT_50A, SHUNT_200A, SHUNT_300A)
     * @param dePin Driver Enable pin untuk RS485
     * @param rePin Receiver Enable pin untuk RS485
     * @return true jika inisialisasi berhasil
     */
    bool begin(Stream &serialPort, uint8_t slaveAddr, uint16_t shuntType, uint8_t dePin, uint8_t rePin);

    /**
     * @brief Mengubah alamat Modbus perangkat
     * @param newAddress Alamat baru (1-247)
     * @return true jika berhasil
     * @note Menggunakan broadcast address 0xF8, semua perangkat di bus akan berubah!
     */
    bool changeAddress(uint8_t newAddress);

    /**
     * @brief Mengatur nilai shunt
     * @param shuntType Tipe shunt (SHUNT_100A, SHUNT_50A, SHUNT_200A, SHUNT_300A)
     * @return true jika berhasil
     */
    bool setShunt(uint16_t shuntType);

    /**
     * @brief Reset counter energi ke nol
     * @return true jika berhasil
     */
    bool resetEnergy();

    /**
     * @brief Membaca tegangan (V)
     * @param value Reference untuk menyimpan hasil
     * @return true jika pembacaan berhasil
     */
    bool readVoltage(float &value);

    /**
     * @brief Membaca arus (A)
     * @param value Reference untuk menyimpan hasil
     * @return true jika pembacaan berhasil
     */
    bool readCurrent(float &value);

    /**
     * @brief Membaca daya (W)
     * @param value Reference untuk menyimpan hasil
     * @return true jika pembacaan berhasil
     */
    bool readPower(float &value);

    /**
     * @brief Membaca energi (kWh)
     * @param value Reference untuk menyimpan hasil
     * @return true jika pembacaan berhasil
     */
    bool readEnergy(float &value);

    /**
     * @brief Membaca semua parameter sekaligus
     * @param voltage Tegangan (V)
     * @param current Arus (A)
     * @param power Daya (W)
     * @param energy Energi (kWh)
     * @return true jika semua pembacaan berhasil
     */
    bool readAll(float &voltage, float &current, float &power, float &energy);

    /**
     * @brief Memeriksa apakah device terhubung
     * @return true jika device merespon
     */
    bool isConnected();

    /**
     * @brief Mendapatkan alamat slave saat ini
     * @return Alamat slave
     */
    uint8_t getSlaveAddress() const { return slaveAddress; }

    /**
     * @brief Memeriksa status inisialisasi
     * @return true jika sudah diinisialisasi
     */
    bool isInitialized() const { return initialized; }
};

#endif // PZEM017_H