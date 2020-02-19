#include <SPI.h>
#include <SD.h>


// Motor driver pins
#define IN1 7
#define IN2 8
#define PWM 9
#define STBY 6

// LED pins
#define LED_1 2
#define LED_2 3
#define LED_3 4
#define LED_4 5

// Hall sensor pins
#define HALL A0

// SD card pins
#define CS 10

//variables
boolean sd_init = false;
File log_file;

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

}

void loop() {

}
