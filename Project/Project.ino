#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"
#include "RF24.h"
#include "printf.h"
#include <SPI.h>

RF24 radio(14, 15);
uint8_t address[][6] = {"1Node", "2Node"};
bool radioNumber     = 0;
bool role            = false;
byte payload;

int incomingByte = 0;

byte incomingBytes[10];

byte lastJoystickValue = 0;

typedef void (*commandBody)(byte[10]);

typedef struct
{
    int requiredBytes;
    commandBody body;
} command;

command commandLookup[256];

DynamixelInterface dInterface(Serial2, 2);
DynamixelMotor servos[4] = {
    DynamixelMotor(dInterface, 0),
    DynamixelMotor(dInterface, 1),
    DynamixelMotor(dInterface, 2),
    DynamixelMotor(dInterface, 3),
};

int motors[4] = {
    23, 22, 21, 20};

unsigned long lastUpdate;

void handleSetServoTargetDegree(byte incomingBytes[10])
{
    Serial.println("Servo target position");
    uint8_t id        = incomingBytes[1];
    uint16_t position = (incomingBytes[3] << 8) + incomingBytes[2];

    servos[id].goalPositionDegree(position);
}

void handleSetServoLight(byte incomingBytes[10])
{
    Serial.println("Servo light");
    uint8_t id = incomingBytes[1];
    bool state = incomingBytes[2];

    servos[id].led(state);
}

void handleSetServoSpeed(byte incomingBytes[10])
{
    Serial.println("Servo speed");
    uint8_t id     = incomingBytes[1];
    uint16_t speed = (incomingBytes[3] << 8) + incomingBytes[2];

    servos[id].speed(speed);
}

void handleSetMotorPwm(byte incomingBytes[10])
{
    Serial.println("Motor pwm");
    uint8_t id     = incomingBytes[1];
    uint16_t speed = incomingBytes[2];

    Serial.println(speed);
    analogWrite(motors[id], speed);
}

void setup()
{
    Serial.begin(115200);
    // while (!Serial) {}
    if (!radio.begin())
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {}
    }
    Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
    //while (!Serial.available()) {}
    //char input  = Serial.parseInt();
    radioNumber = false;

    Serial.print(F("radioNumber = "));
    Serial.println((int)radioNumber);
    Serial.println(F("*** PRESS 'T' to begin transmitting to the other node"));

    Serial.print(F("Payload size="));
    Serial.println(radio.getPayloadSize());

    radio.setPALevel(RF24_PA_MAX);
    radio.setPayloadSize(1);
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
    commandLookup[0x01].body          = &handleSetServoTargetDegree;

    commandLookup[0x02].requiredBytes = 3;
    commandLookup[0x02].body          = &handleSetServoLight;

    commandLookup[0x03].requiredBytes = 4;
    commandLookup[0x03].body          = &handleSetServoSpeed;

    commandLookup[0x04].requiredBytes = 3;
    commandLookup[0x04].body          = &handleSetMotorPwm;

    dInterface.begin(115200, 50);

    for (byte i = 0; i < 4; i++)
    {
        servos[i].init();
        servos[i].statusReturnLevel(2);
        servos[i].jointMode();
        servos[i].enableTorque();
        servos[i].speed(1023);
        servos[i].torqueLimit(1023);
    }

    for (int i = 0; i < 4; i++) {
        analogWrite(motors[i], LOW);
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
            Serial.println("New command");
            command = commandLookup[incomingByte];
        }
        else
        {
            incomingBytes[i] = incomingByte;
        }

        Serial.println(incomingByte);
        i++;

        if (i == command.requiredBytes)
        {
            Serial.println("Command done");
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

    if (role)
    {
        // TODO: Send packets
    }
    else
    {
        uint8_t pipe;

        if (radio.available(&pipe))
        {
            uint8_t bytes = radio.getPayloadSize();

            radio.read(&payload, bytes);

            if ((payload & 0b01000000) == 0b01000000)
            {
                lastJoystickValue = payload & 0b00111111;
            }

            // servos[0].goalPosition(map(payload[0], 0, 1023, 1023 - 120, 1023 - 560));
            // servos[1].goalPosition(map(payload[1], 0, 1023, 120, 560));
            // servos[2].goalPosition(map(payload[2], 0, 1023, 120, 560));
            // servos[3].goalPosition(map(payload[3], 0, 1023, 1023 - 120, 1023 - 560));
            // servos[1].goalPosition(payload[1]);
            // servos[2].goalPosition(payload[2]);
            // servos[3].goalPosition(payload[3]);
        }
    }

    if (millis() - lastUpdate > 100)
    {
        // Motors
        for (uint8_t i = 0; i < 4; i++)
        {
            uint16_t pos;
            DynamixelStatus status = servos[0].currentPosition(pos);

            uint8_t* _pos = (uint8_t*)&pos; // The fuck

            uint8_t data[] = {0x00, i, _pos[0], _pos[1]};

            Serial3.write(data, 4);
        }

        // Joystick
        {
            uint8_t data[] = {0x01, lastJoystickValue};

            Serial3.write(data, 2);
        }
        //Serial3.flush();

        lastUpdate = millis();
    }
}
