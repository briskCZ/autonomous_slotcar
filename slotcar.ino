#include <SPI.h>
#include <SD.h>
#include <Arduino_LSM9DS1.h>

// Motor driver
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// LED
#define LED_1 2
#define LED_2 3
#define LED_3 4
#define LED_4 5

// Hall sensor
#define HALL A0
#define TRESHOLD_HIGH 500
#define TRESHOLD_LOW 180

#define TRACK_LENGTH 425

// SD card
#define CS 10

// Variables
int state = 0;  // 0 - first slow lap, 1 = fast driving

class SdCard{
    private:
       bool initialized = false;
       File file;
    
    public:
        void setup(){
            if(SD.begin(CS)){
                initialized = true;
            }
        }

        void write(String string){
            if(initialized){
                file = SD.open("log_file.txt", FILE_WRITE);
                file.print(string);
                file.close();                
            }
        }
};

class Motor {
    private:
        int current_speed;

    public:

        void setup(){
            pinMode(IN1, OUTPUT);
            pinMode(IN2, OUTPUT);
            pinMode(PWM, OUTPUT);
            pinMode(STBY, OUTPUT);
        }

        void drive(int speed){
            if(speed != current_speed){
                digitalWrite(STBY, HIGH);
                digitalWrite(IN1, HIGH);
                digitalWrite(IN2, LOW);
                analogWrite(PWM, speed);
            }
            current_speed = speed;
        }

        void brake(){
            digitalWrite(IN1, HIGH);
            digitalWrite(IN2, HIGH);
            analogWrite(PWM,0);
            current_speed = 0;
        }

        void turnOff(){
            analogWrite(PWM,0);
            digitalWrite(STBY, LOW);
        }

};

class Hall {
    private:
        int value;
        bool can_count = true;
        int rotations = 0;
        const float tire_circumference = 7.4;   // All lenghts are in cm
        int traveled_distance = 0;
    
    public:
    void loop(){
        value = analogRead(HALL);

        if(value > TRESHOLD_HIGH && can_count){
            rotations ++;
            traveled_distance = rotations * tire_circumference;
            can_count = false;
        }

        if(value < TRESHOLD_LOW){
            can_count = true;
        }
    }

    int getDistance(){
        return traveled_distance;
    }

};

// TODO: class Led ....

Motor motor;
SdCard sd;
Hall hall;

void setup() {
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);

    motor.setup();
    sd.setup();

    sd.write("================================ NEW LOG ================================\n\n");

    if (!IMU.begin()) {
        sd.write("IMU inicialization failed!");
        // TODO: do something with lights to signal failure
    }

}

void loop() {

    hall.loop();

    if(hall.getDistance() > TRACK_LENGTH){
        state = 1;
    }

    if(state == 0){
        motor.drive(45);

        float x,y,z;    // x - front and back, y - left and right, z - up and down
        if (IMU.accelerationAvailable()) {
            IMU.readAcceleration(x, y, z);
        }
    }

    if(state == 1){
        motor.brake();
    }

}
