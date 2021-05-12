#include "Dynamixel.h"
#include "DynamixelInterface.h"
#include "DynamixelMotor.h"

int incomingByte = 0;

byte incomingBytes[10];

typedef void (*commandBody)(byte[10]);

typedef struct
{
    int requiredBytes;
    commandBody body;
} command;

command commandLookup[256];

DynamixelInterface dInterface(Serial2, 2); // Stream
DynamixelMotor motor(dInterface, 0);

void handleSetServoTargetPosition(byte incomingBytes[10])
{
    short position = (incomingBytes[3] << 8) + incomingBytes[2];

	Serial.println("Servo position");
	Serial.println(position);

    motor.goalPositionDegree(position);
}

void handleSetServoLight(byte incomingBytes[10])
{
	Serial.println("Servo light");
	bool state = incomingBytes[2];

	Serial.println(state);

	motor.led(state);
}

void setup()
{
	Serial.begin(9600);
    Serial.println("Starting the Serial3");

	Serial3.begin(9600);

	commandLookup[0x01].requiredBytes = 4;
    commandLookup[0x01].body          = &handleSetServoTargetPosition;

    commandLookup[0x02].requiredBytes = 3;
    commandLookup[0x02].body          = &handleSetServoLight;

    pinMode(13, OUTPUT);

    
    dInterface.begin(115200, 50);                // baudrate, timeout
    
    //motor = DynamixelMotor(dInterface, 0); // Interface , ID

    //ctx.motor = motor;

    motor.init(); // This will get the returnStatusLevel of the servo
    Serial.printf("Status return level = %u\n", motor.statusReturnLevel());
    motor.jointMode(); // Set the angular limits of the servo. Set to [min, max] by default
    motor.enableTorque();

    

    // // DynamixelInterface dInterface(Serial2, 2); // Stream
    // // dInterface.begin(9600, 50);                // baudrate, timeout
    // // motor = new DynamixelMotor(dInterface, 0); // Interface , ID

    // motor->init(); // This will get the returnStatusLevel of the servo

    // motor->jointMode(); // Set the angular limits of the servo. Set to [min, max] by default
    // motor->enableTorque();

    // Serial.write("Status\n");
    // DynamixelStatus status = motor->ping();
    // Serial.println(status);

    
}

bool ledState = LOW;

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
    //Serial.println("Reading the Serial3");

    digitalWrite(13, ledState);
    ledState = !ledState;

    while (Serial3.available() > 0)
    {
        handleCommand();
    }
}