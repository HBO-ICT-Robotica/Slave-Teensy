#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"
#include "RF24.h"
#include "printf.h"
#include <SPI.h>

RF24 radio(14, 15);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber     = 1;
bool role            = false;
uint16_t payload[4]        = { };

int incomingByte = 0;

byte incomingBytes[10];

typedef void (*commandBody)(byte[10]);

typedef struct
{
    int requiredBytes;
    commandBody body;
} command;

command commandLookup[256];

DynamixelInterface dInterface(Serial2, 2);
DynamixelMotor motors[4] = {
    DynamixelMotor(dInterface, 0),
    DynamixelMotor(dInterface, 1),
    DynamixelMotor(dInterface, 2),
    DynamixelMotor(dInterface, 3),
};

unsigned long lastUpdate;

void handleSetServoTargetPosition(byte incomingBytes[10])
{
    Serial.println("Servo target position");
    uint8_t id        = incomingBytes[1];
    uint16_t position = (incomingBytes[3] << 8) + incomingBytes[2];

    motors[id].goalPosition(position);
}

void handleSetServoLight(byte incomingBytes[10])
{
    Serial.println("Servo light");
    uint8_t id = incomingBytes[1];
    bool state = incomingBytes[2];

    motors[id].led(state);
}

void handleSetServoSpeed(byte incomingBytes[10])
{
    Serial.println("Servo speed");
    uint8_t id     = incomingBytes[1];
    uint16_t speed = (incomingBytes[3] << 8) + incomingBytes[2];

    motors[id].speed(speed);
}

void setup()
{
    Serial.begin(115200);
    while (!Serial) {}
    if (!radio.begin())
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {}
    }
    Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
    while (!Serial.available()) {}
    char input  = Serial.parseInt();
    radioNumber = input == 1;

    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(sizeof(payload));
    radio.openWritingPipe(address[radioNumber]);
    radio.openReadingPipe(1, address[!radioNumber]);

    Serial.print(F("radioNumber = "));
    Serial.println((int)radioNumber);
    Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

    Serial.print(F("Payload size="));
    Serial.println(radio.getPayloadSize());

    radio.setPALevel(RF24_PA_LOW);
    radio.setPayloadSize(8);
    radio.openWritingPipe(address[radioNumber]);
    radio.openReadingPipe(1, address[!radioNumber]);

    if (role)
    {
        radio.stopListening();
    }
    else
    {
        radio.startListening();
    }

    Serial3.begin(9600);

    commandLookup[0x01].requiredBytes = 4;
    commandLookup[0x01].body          = &handleSetServoTargetPosition;

    commandLookup[0x02].requiredBytes = 3;
    commandLookup[0x02].body          = &handleSetServoLight;

    commandLookup[0x03].requiredBytes = 4;
    commandLookup[0x03].body          = &handleSetServoSpeed;

    dInterface.begin(115200, 50);

    for (byte i = 0; i < 4; i++)
    {
        motors[i].init();
        motors[i].statusReturnLevel(2);
        motors[i].jointMode();
        motors[i].enableTorque();
        motors[i].speed(1023);
        motors[i].torqueLimit(1023);
    }
}

void handleCommand()
{
    int i = 0;

    command command;

    while (true)
    {
        if (Serial3.available() == 0)
        {
            continue;
        }

        int incomingByte = Serial3.read();

        if (i == 0)
        {
            command = commandLookup[incomingByte];
        }
        else
        {
            incomingBytes[i] = incomingByte;
        }

        i++;

        if (i == command.requiredBytes)
        {
            break;
        }
    }

    command.body(incomingBytes);
}

void loop()
{
    while (Serial3.available() > 0)
    {
        handleCommand();
    }

    if (role) {
        // TODO: Send packets
    } else {
        uint8_t pipe;

        if (radio.available(&pipe)) {
            uint8_t bytes = radio.getPayloadSize();

            radio.read(&payload, bytes);

            Serial.print("Received ");
            Serial.println(payload[0]);
            Serial.println(payload[1]);
            Serial.println(payload[2]);
            Serial.println(payload[3]);

            motors[0].goalPosition(map(payload[0], 0, 1023, 1023 - 120, 1023 - 560));
            motors[1].goalPosition(map(payload[1], 0, 1023, 120, 560));
            motors[2].goalPosition(map(payload[2], 0, 1023, 120, 560));
            motors[3].goalPosition(map(payload[3], 0, 1023, 1023 - 120, 1023 - 560));
            // motors[1].goalPosition(payload[1]);
            // motors[2].goalPosition(payload[2]);
            // motors[3].goalPosition(payload[3]);
        }
    }

    if (millis() - lastUpdate > 100)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            uint16_t pos;
            DynamixelStatus status = motors[0].currentPosition(pos);

            uint8_t* _pos = (uint8_t*)&pos; // The fuck

            uint8_t data[] = {0x00, i, _pos[0], _pos[1]};

            Serial3.write(data, 4);
        }
        //Serial3.flush();

        lastUpdate = millis();
    }
}
