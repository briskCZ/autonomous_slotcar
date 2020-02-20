#include <SPI.h>
#include <SD.h>


// Motor driver pins
#define STBY 6
#define IN1 7
#define IN2 8
#define PWM 9

// LED pins
#define LED_1 2
#define LED_2 3
#define LED_3 4
#define LED_4 5

// Hall sensor pins
#define HALL A0

// SD card pins
#define CS 10

// Variables
boolean sd_init = false;
File log_file;

int hall_value;
bool can_count = true;
int rotations = 0;
int track_length = 425;   // cm
float tire_travel = 7.4;    // cm

int state = 0;  // 0 - first slow lap, 1 = fast driving


void drive(int speed){
    digitalWrite(STBY, HIGH);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PWM, speed);
}

void brake(){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, HIGH);
    analogWrite(PWM,0);
}

void motorOff(){
    analogWrite(PWM,0);
    digitalWrite(STBY, LOW);
}

void setup() {

    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(PWM, OUTPUT);
    pinMode(STBY, OUTPUT);

    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);
    pinMode(LED_4, OUTPUT);


    if(SD.begin(CS)){
        sd_init = true;
    }

    if(sd_init){
        log_file = SD.open("log_file.txt", FILE_WRITE);
        log_file.println("================================ NEW LOG ================================\n");
        log_file.close();
    }

    drive(45);

}

void loop() {

    hall_value = analogRead(HALL);

    if(hall_value > 500 && can_count){
        rotations ++;
        if(sd_init){
            log_file = SD.open("log_file.txt", FILE_WRITE);
            log_file.print("Rotations: ");
            log_file.println(rotations);
            log_file.close();
        }
        can_count = false;
    }

    if(hall_value < 180){
        can_count = true;
    }

    float traveled_distance = tire_travel * rotations;

    if(traveled_distance > track_length){
        brake();
    }

}
