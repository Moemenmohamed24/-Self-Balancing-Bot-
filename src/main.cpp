#include <Arduino.h>

/********** PID **********/
#include <PID_v1.h>

double kP = 16;
// double kI = 350;
double kI = 50;
double kD = 0.4;

double setpoint, input, output;   // PID variables
PID pid(&input, &output, &setpoint, kP, kI, kD, DIRECT); // PID setup

/**********Bluetooth*********/
#include <BluetoothSerial.h>
BluetoothSerial SerialBT;
float angleV = 0, turnV = 0; // values from remote controller

/********** L293D **********/
#include <L293D.h>

// Pin definition
#define EN_A 32
// #define IN1_A 14
// #define IN2_A 27

#define IN1_A 27
#define IN2_A 13 

// #define IN3_B 25
// #define IN4_B 26

#define IN3_B 26
#define IN4_B 25
#define EN_B 33

#define PWM_MOTOR_FREQUENCY   100
#define PWM_MOTOR_RESOLUTION    8


L293D rightmotor(IN1_A,IN2_A,EN_A,0);
L293D leftmotor(IN3_B,IN4_B,EN_B,1);

// motor speeds
float MULL = 1.5;
float speedLeft = 0;
float speedRight = 0;


/*********DotMatrix*******/
#include <MD_MAX72xx.h>
#include <SPI.h>
#include "MD_EyePair.h"

// Define the number of devices (8x8 LED matrices) cascaded
#define MAX_DEVICES 2

// Define the type of hardware. Use FC16_HW for generic MAX7219 modules
#define HARDWARE_TYPE MD_MAX72XX::FC16_HW

// Define the ESP32 pins for SPI communication
#define DATA_PIN 23  // DIN pin 
#define CS_PIN   5  // CS pins of both modules 
#define CLK_PIN  18  // CLK pin 

#define MAX_EYE_PAIR (MAX_DEVICES/2)

int INTENSITY = 2;

MD_MAX72XX mx=MD_MAX72XX(HARDWARE_TYPE,CS_PIN,MAX_DEVICES) ;

MD_EyePair E[MAX_EYE_PAIR];

#define	DELAYTIME  500  // in milliseconds

/********** MPU **********/
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

#define Interrupt_PIN 15

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float pitch;

int IMUdataReady = 0;
volatile bool mpuInterrupt = false;

void dmpDataReady() {
     IMUdataReady = 1;
}        


// function that actually read the angle when the flag is set by the ISR
void readAngles()  {

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));
     } 
     
     else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        IMUdataReady = 0;
        //count = count + 1;
    }
}

void bluetooth_task(void * parameter) {

/**********Bluetooth*********/

    while(true) {
        if (SerialBT.available()) {
            String command = SerialBT.readStringUntil('\n');
            command.trim();
        
            // Split command into parts
            int spaceIndex = command.indexOf(' ');
            if (spaceIndex == -1) {
                SerialBT.println("Invalid command format");
                continue;
            }
            String key = command.substring(0, spaceIndex);
            String valueStr = command.substring(spaceIndex + 1);
            double value = valueStr.toDouble();
            // Update variables based on command
            if (key == "p") {
                kP = value;
                pid.SetTunings(kP,kI,kD); 
                SerialBT.println("kP updated to: ");
                SerialBT.println(value);
            } else if (key=="tune") {
                switch (int(value))
                {
                case 1:
                    kP=24;
                    kD=1.2;
                    kI=50;
                    pid.SetTunings(kP,kI,kD); 
                    SerialBT.println("All The PID tunings has been reset.");
                    SerialBT.printf("KP=%.4f, KD=%.4f, KI=%.4f \n",kP,kD,kI);
                    break;
                case 2:
                    kP=16;
                    kD=1.2;
                    kI=50;
                    pid.SetTunings(kP,kI,kD); 
                    SerialBT.println("All The PID tunings has been changed to:-");
                    SerialBT.printf("KP=%.4f, KD=%.4f, KI=%.4f \n",kP,kD,kI);
                    break;
                case 3:
                    kP=35;
                    kD=1.249;
                    kI=70;
                    pid.SetTunings(kP,kI,kD); 
                    SerialBT.println("All The PID tunings has been changed to:-");
                    SerialBT.printf("KP=%.4f, KD=%.4f, KI=%.4f \n",kP,kD,kI);
                    break;
                case 4:
                    kP=20;
                    kD=1.3;
                    kI=40;
                    pid.SetTunings(kP,kI,kD); 
                    SerialBT.println("All The PID tunings has been changed to:-");
                    SerialBT.printf("KP=%.4f, KD=%.4f, KI=%.4f \n",kP,kD,kI);
                    break;
                default:
                    break;
                }
            } else if (key == "d") { 
                kD = value;
                pid.SetTunings(kP,kI,kD); 
                SerialBT.print("kD updated to: ");
                SerialBT.println(value);
            } else if (key == "i") {
                kI = value;
                pid.SetTunings(kP,kI,kD); 
                SerialBT.print("kI updated to: ");
                SerialBT.println(value);
            } else if (key == "set") {
                setpoint = value;
                SerialBT.print("Setpoint updated to: ");
                SerialBT.println(value);
            } else if (key == "bright") {
                INTENSITY = value;
                mx.control(MD_MAX72XX::INTENSITY,INTENSITY);
                SerialBT.print("Intensity updated to: ");
                SerialBT.println(value);
            } else if (key == "mull") {
                MULL = value;
                SerialBT.print("Mull updated to: ");
                SerialBT.println(value);
            } else if(key == "show") {
                SerialBT.printf("The current angle is: %.4f \n",pitch);
                SerialBT.printf("The current output is: %.4f \n",speedLeft);
                SerialBT.printf("The current Setpoint is: %.4f \n",setpoint);
                SerialBT.printf("The current kP is: %.4f \n",kP);
                SerialBT.printf("The current kD is: %.4f \n",kD);
                SerialBT.printf("The current kI is: %.4f \n",kI);
                SerialBT.printf("The current MULL is: %.4f \n",MULL);
                SerialBT.printf("The current intensity is: %.4f \n",INTENSITY);
            } else if (key=="mreset") {
                mpu.resetFIFO();
                SerialBT.println("The FIFO buffer has been reset.");
            } else if (key=="reset") {
                kP=16;
                kD=0.4;
                kI=50;
                pid.SetTunings(kP,kI,kD); 
                SerialBT.println("All The PID tunings has been reset.");
                SerialBT.printf("KP=%.4f, KD=%.4f, KI=%.4f \n",kP,kD,kI);
            } else if (key=="help") {
                SerialBT.println("\
                   Available command are:- \n \
                   p <value> to edit kP.\n \
                   d <vlaue> to edit kD.\n \
                   i <value> to edit kI.\n \
                   set <value> to edit the setpoint.\n \
                   mull <vlaue> to edit the speed multiplier.\n \
                   bright <vlaue> to edit the brightness of the dot matrix.\n \
                   mreset <any number> to reset the FIFO buffer.\n \
                   reset <any number> to reset the tuning values.\n \
                   show <any number> to show the current values and states.");
            } else {
                SerialBT.println("Unknown command");
            }
            
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}


void dot_matrix_task(void * parameter) {
    while (true)
    {
        for (uint8_t i=0; i<MAX_EYE_PAIR; i++) {
            E[i].animate();
        }
    } 
}



void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(115200);

/**********Bluetooth*********/
    if (!SerialBT.begin("ESP32_Controller")) { // Bluetooth device name
        Serial.println("Bluetooth initialization failed!");
    }
    Serial.println("Bluetooth device is ready to pair");
    xTaskCreatePinnedToCore(
        bluetooth_task,
        "Bluetooth",
        8192,
        NULL,
        1,
        NULL,
        0
    );

/********** MPU **********/
    setpoint = 1;
    // initialize device
    mpu.initialize();
    devStatus = mpu.dmpInitialize();

    // gyro offsets
    mpu.setXGyroOffset(158);
    mpu.setYGyroOffset(31);
    mpu.setZGyroOffset(6);
    mpu.setZAccelOffset(713);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        mpu.setDMPEnabled(true);

        // enable ESP interrupt detection
        attachInterrupt(digitalPinToInterrupt(Interrupt_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
    
    pid.SetMode(AUTOMATIC);
    pid.SetOutputLimits(-100,100);
    pid.SetSampleTime(10);

    /**********Motors**********/
    rightmotor.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    leftmotor.begin(true, PWM_MOTOR_FREQUENCY, PWM_MOTOR_RESOLUTION);
    
    rightmotor.SetMotorSpeed(0);
    leftmotor.SetMotorSpeed(0);
    /*************DotMatrix*******/
    mx.begin();
    mx.control(MD_MAX72XX::INTENSITY,INTENSITY);

     // initialize the eye view
    for (uint8_t i=0; i<MAX_EYE_PAIR; i++){
        E[i].begin(i*2, &mx, DELAYTIME);
    }
    xTaskCreatePinnedToCore(
        dot_matrix_task,
        "Dot_Matrix",
        8192,
        NULL,
        0,
        NULL,
        0
    );
}

void loop() {
    if (IMUdataReady == 1) {
        readAngles();
    }
    
    pitch = (ypr[1] * 180/M_PI); // adjust to degrees

    // reset the FIFO buffer in case of gimbal lock
    if (abs(pitch) >= 95) {
        mpu.resetFIFO();
    }
    
    // PID vars
    setpoint += angleV; 
    input = pitch;
    
    
    pid.Compute();
    
    // set motor speed with adjusted turn values
    speedLeft = output -  turnV;
    speedRight = output + turnV;    
    
    // move motors
    rightmotor.SetMotorSpeed(speedRight*MULL);
    leftmotor.SetMotorSpeed(speedLeft);

}
