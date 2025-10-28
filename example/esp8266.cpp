#include <PZEM017.h>

#define RE D1
#define DE D2

#define RS485_RX D5
#define RS485_TX D6

PZEM017 pzem_dc;

void setup()
{
    Serial.begin(115200);

    pzem_dc(RS485_RX, RS485_TX);
    pzem_dc.begin(0x01, SHUNT_50A, DE, RE);

    delay(1000);
}

void loop()
{
    Serial.print("Voltage: ");
    Serial.println(pzem_dc.readVoltage());

    Serial.print("Current: ");
    Serial.println(pzem_dc.readCurrent());

    Serial.print("Power: ");
    Serial.println(pzem_dc.readPower());

    Serial.print("Energy: ");
    Serial.println(pzem_dc.readEnergy());

    delay(1000);
}