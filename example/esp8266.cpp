/**
 * @file example_usage.ino
 * @brief Contoh penggunaan library PZEM017 refactored
 */

#include <PZEM017.h>

// ========== Configuration ==========
// Gunakan HardwareSerial (recommended untuk ESP32)
#define PZEM_SERIAL Serial2
#define PZEM_RX_PIN 16
#define PZEM_TX_PIN 17

// RS485 Control Pins
#define RS485_DE_PIN 4
#define RS485_RE_PIN 5

// Modbus Configuration
#define PZEM_SLAVE_ADDR 0x01
#define PZEM_SHUNT_TYPE SHUNT_100A

// ========== Global Objects ==========
PZEM017 pzem;

void setup()
{
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== PZEM017 Refactored Demo ===");

    // Setup HardwareSerial
    PZEM_SERIAL.begin(9600, SERIAL_8N1, PZEM_RX_PIN, PZEM_TX_PIN);

    // Initialize PZEM017
    Serial.print("Initializing PZEM017... ");
    if (pzem.begin(PZEM_SERIAL, PZEM_SLAVE_ADDR, PZEM_SHUNT_TYPE, RS485_DE_PIN, RS485_RE_PIN))
    {
        Serial.println("OK");
    }
    else
    {
        Serial.println("FAILED!");
        Serial.println("Check wiring and power supply");
        while (1)
            delay(1000);
    }

    // Test connection
    Serial.print("Testing connection... ");
    if (pzem.isConnected())
    {
        Serial.println("Device found!");
    }
    else
    {
        Serial.println("Device not responding!");
    }

    Serial.println("\nStarting measurements...\n");
}

void loop()
{
    float voltage, current, power, energy;

    // Metode 1: Baca semua sekaligus
    if (pzem.readAll(voltage, current, power, energy))
    {
        Serial.println("=== PZEM017 Readings ===");
        Serial.print("Voltage: ");
        Serial.print(voltage, 2);
        Serial.println(" V");
        Serial.print("Current: ");
        Serial.print(current, 3);
        Serial.println(" A");
        Serial.print("Power:   ");
        Serial.print(power, 1);
        Serial.println(" W");
        Serial.print("Energy:  ");
        Serial.print(energy, 3);
        Serial.println(" kWh");
        Serial.println();
    }
    else
    {
        Serial.println("ERROR: Failed to read data!");
    }

    /*
    // Metode 2: Baca individual dengan error handling
    float voltage;
    if (pzem.readVoltage(voltage)) {
        Serial.print("Voltage: "); Serial.print(voltage, 2); Serial.println(" V");
    } else {
        Serial.println("Failed to read voltage");
    }
    */

    delay(2000); // Update setiap 2 detik
}

// ========== Utility Functions ==========

/**
 * @brief Reset energy counter (panggil via serial command)
 */
void resetEnergyCounter()
{
    Serial.print("Resetting energy counter... ");
    if (pzem.resetEnergy())
    {
        Serial.println("OK");
    }
    else
    {
        Serial.println("FAILED");
    }
}

/**
 * @brief Change device address (uncomment jika perlu)
 */
/*
void changeDeviceAddress(uint8_t newAddr) {
    Serial.print("Changing address to 0x");
    Serial.print(newAddr, HEX);
    Serial.print("... ");

    if (pzem.changeAddress(newAddr)) {
        Serial.println("OK");
        Serial.println("Please update PZEM_SLAVE_ADDR and reboot");
    } else {
        Serial.println("FAILED");
    }
}
*/