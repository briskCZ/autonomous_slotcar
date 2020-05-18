//  Autonomous slotcar - uses Arduino Nano 33 BLE
//  Bachelor thesis
//  Author: Marek Nesvadba (xnesva06)
//  FIT Brno university of technology
//  2020

// Constant speed version

#include <SPI.h>
#include <SD.h>

// Modified version with function that returns data as integer instead of float
#include <Arduino_LSM9DS1.h>

// Motor driver pins
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// Hall sensor
#define HALL A0
#define TRESHOLD_HIGH 500
#define TRESHOLD_LOW 180

// Accelerometer calibration values
#define N_AVG_SAMPLES 16
#define N_CAL_SAMPLES 100

// Algorithm values
#define CONSTANT_SPEED 75

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
        
        // All lenghts are in cm
        float base_circumference = 7.2;
        float tire_circumference_error = 0.0;
        float traveled_distance = 0;
        float tire_circumference = base_circumference + tire_circumference_error;
    
    public:
    void loop(){
        value = analogRead(HALL);

        if(value > TRESHOLD_HIGH && can_count){
            rotations ++;
            traveled_distance = countDistance(rotations);
            can_count = false;
        }

        if(value < TRESHOLD_LOW){
            can_count = true;
        }
    }

    int getRotations(){
        return rotations;
    }

    void setRotations(int number){
        rotations = number;
    }

    int getTraveledDistance(){
        return traveled_distance;
    }

    int countDistance(int turns){
        return turns * tire_circumference;
    }

    int getValue(){
        return value;
    }

};

// Variables
int state = 0;

Motor motor;
Hall hall;

void setup() {

    // Do not run on table
    if (analogRead(HALL) > 500){
        state = -1;
    }

    motor.setup();
}

void loop() {

    if(state == 0){

        motor.drive(CONSTANT_SPEED);

    }

}
