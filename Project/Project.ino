// #include "Dynamixel.h"
// #include "DynamixelInterface.h"
// #include "DynamixelMotor.h"

// DynamixelInterface dInterface(Serial2, 2);

// DynamixelMotor servo(dInterface, 0);

// void setup()
// {
//     dInterface.begin(115200, 50);

//     servo.init();
//     servo.statusReturnLevel(2);
//     servo.jointMode();
//     servo.enableTorque();
//     servo.speed(1023);
//     servo.torqueLimit(1023);

//     while (true)
//     {
//         DynamixelStatus status = servo.ping();
//         Serial.println(status, BIN);

//         //servo.goalPosition(600);

//         delay(1000);
//     }
// }

// void loop() {

// }

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

byte lastJoystickXValue = 0;
byte lastJoystickYValue = 0;

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

struct Motor
{
    int inAPin;
    int inBPin;
    int pwmPin;

    bool forward;
    byte speed;

    Motor(int inAPin, int inBPin, int pwmPin) {
        this->inAPin = inAPin;
        this->inBPin = inBPin;
        this->pwmPin = pwmPin;

        this->forward = true;
        this->speed = 0;
    }
};

Motor motors[4] = {
    Motor(23, 22, 3),
    Motor(21, 20, 4),
    Motor(19, 18, 5),
    Motor(17, 16, 6),
};

unsigned long lastUpdate;

unsigned long lastDataFromRemoteReceivedTime;

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
    uint8_t speed = incomingBytes[2];

    motors[id].speed = speed;
}

void handleSetMotorMode(byte incomingBytes[10]) {
    Serial.println("Motor mode");

    uint8_t id   = incomingBytes[1];
    uint8_t mode = incomingBytes[2];

    if (mode == 1) {
        // Forward
        motors[id].forward = true;
    } else if (mode == 2) {
        // Reverse
        motors[id].forward = false;
    }
}

void setup()
{
    Serial.begin(115200);
    if (!radio.begin())
    {
        Serial.println(F("radio hardware is not responding!!"));
        while (1) {}
    }
    while (!Serial) { }
    Serial.println(F("Which radio is this? Enter '0' or '1'. Defaults to '0'"));
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
        DynamixelStatus status = servos[i].ping();
        Serial.println(status);

        servos[i].init();
        servos[i].statusReturnLevel(2);
        servos[i].jointMode();
        servos[i].enableTorque();
        servos[i].speed(1023);
        servos[i].torqueLimit(1023);
    }

    for (int i = 0; i < 4; i++)
    {
        pinMode(motors[i].inAPin, OUTPUT);
        pinMode(motors[i].inBPin, OUTPUT);
        pinMode(motors[i].pwmPin, OUTPUT);

        digitalWrite(motors[i].inAPin, LOW);
        digitalWrite(motors[i].inBPin, LOW);
        analogWrite(motors[i].pwmPin, 0);
    }

    Serial.println("init done");

    while (!Serial3.available()) {
        Serial3.read();
    }

    Serial.println("handshake done");
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

        byte incomingByte = Serial3.read();
        Serial.println(incomingByte);

        if (i == 0)
        {
            Serial.println("New command");
            command = commandLookup[incomingByte];
        }
        else
        {
            incomingBytes[i] = incomingByte;
        }

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

            lastDataFromRemoteReceivedTime = millis();

            radio.read(&payload, bytes);

            if ((payload & 0b10000000) == 0b00000000)
            {
                byte id = payload & 0b01000000;

                if (id == 0)
                {
                    lastJoystickXValue = payload & 0b00111111;
                }
                else
                {
                    lastJoystickYValue = payload & 0b00111111;
                }
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

    for (int i = 0; i < 4; i++) {
        if (motors[i].forward) {
            digitalWrite(motors[i].inAPin, HIGH);
            digitalWrite(motors[i].inBPin, LOW);
        } else {
            digitalWrite(motors[i].inAPin, LOW);
            digitalWrite(motors[i].inBPin, HIGH);
        }

        analogWrite(motors[i].pwmPin, motors[i].speed);
    }

    if (millis() - lastUpdate > 1000)
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

        // Joystick X
        {
            uint8_t data[] = {0x01, 0x00, lastJoystickXValue};
            Serial3.write(data, 3);
        }

        // Joystick Y
        {
            uint8_t data[] = {0x01, 0x01, lastJoystickYValue};
            Serial3.write(data, 3);
        }
        lastUpdate = millis();
    }

    if (millis() - lastDataFromRemoteReceivedTime > 100) {
        uint8_t data[] = {0x02};
        Serial3.write(data, 1);
    }
}
